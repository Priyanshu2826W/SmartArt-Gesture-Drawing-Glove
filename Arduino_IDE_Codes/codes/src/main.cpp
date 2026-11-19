// =============================================================================
//  IMU Pen Tracker v3 — Quaternion Orientation + 3-D Dead-Reckoning + 2-D Projection
//  ESP32-S2 + BNO080
//
//  ── ALGORITHM ────────────────────────────────────────────────────────────────
//
//  Step 1  ROTATION VECTOR — QUATERNION (BNO080 on-chip sensor fusion)
//    enableRotationVector(20) asks the BNO080 to run its internal AHRS
//    (accelerometer + gyroscope + magnetometer Kalman filter) and output a
//    unit quaternion q that maps body frame → world frame.
//    World frame: Z is UP (away from gravity), XY is horizontal.
//    This is DRIFT-FREE for orientation — no manual gyro integration needed.
//
//  Step 2  BODY → WORLD FRAME ACCELERATION  (every IMU tick)
//    a_world = q ⊗ a_body ⊗ q*        (quaternion sandwich product)
//    Implemented via the efficient Rodriguez formula:
//        t = 2 * (q_vec × v)
//        v' = v + q_w*t + (q_vec × t)
//
//  Step 3  GRAVITY SUBTRACTION
//    When stationary the accelerometer reads the support force = +G_MS2 in
//    the world Z axis.  Linear accel = a_world − [0, 0, G_MS2].
//
//  Step 4  LEVER ARM CORRECTION (body frame)
//    IMU sits between wrist and MCP; pen tip is 12 cm further along body Y.
//    a_pen = a_IMU + (α × r) + (ω × (ω × r))    where r = {0, 0.12, 0} m
//
//  Step 5  DRIFT SUPPRESSION
//    • ZUPT  — hard velocity reset when |a_lin| < 0.08 m/s² AND |ω| < 0.03 rad/s
//    • Bias estimation — slow EMA of linear accel during ZUPT
//    • Dead-band — accelerations below 0.05 m/s² zeroed
//    • Velocity cap — max 0.50 m/s
//
//  Step 6  KAHAN COMPENSATED 3-D INTEGRATION  (Knuth Vol.2 §4.2.2, Alg. A)
//    Velocity and position maintained in three independent Kahan accumulators
//    (world X, Y, Z).  Gives near-double-precision summation in float.
//
//  Step 7  2-D PROJECTION ONTO PAPER PLANE
//    At calibration ('z'), the current world XY plane is defined as the paper.
//    Project: (x_2d, y_2d) = (posX_world, posY_world) in mm.
//    For a tilted paper, paper_x_axis / paper_y_axis can be rotated
//    (see startGravityCalib for the general Gram-Schmidt version).
//
//  Step 8  1-D KALMAN FILTER (X and Y independently)
//    Smooths the projected 2-D coordinates before sending.
//
//  ── GESTURE CLASSIFICATION ───────────────────────────────────────────────────
//    Displacement-based (no ML, no weights file):
//      's' → record (roll₀, pitch₀, yaw₀) and reset path length
//      'e' → compare end displacement to gesture_threshold.h → LINE or CIRCLE
//
//  ── SERIAL COMMANDS ──────────────────────────────────────────────────────────
//    z / c  — set world origin + define paper plane (instant, no sampling)
//    s      — start gesture recording
//    e      — end gesture → classify LINE / CIRCLE
//    d      — fit shape to stored XY trajectory
//    r      — reset position (keep orientation)
//    R      — full reset
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>

#include "gesture_threshold.h"

// ─────────────────────────── WiFi & Server ───────────────────────────────────
#define WIFI_SSID    "iQOO Neo7"
#define WIFI_PASS    "sahil@222"
#define SERVER_IP    "10.251.102.77"
#define SERVER_PORT  5001
#define SERVER_PATH  "/ingest"

// ─────────────────────────── Telemetry Rate ──────────────────────────────────
#define SEND_HZ 50
static const uint32_t SEND_INTERVAL_MS = 1000 / SEND_HZ;

// ─────────────────────────── Physical Constants ──────────────────────────────
static const float G_MS2   = 9.81f;
static const float M_TO_MM = 1000.0f;
static const float PAPER_W = 210.0f;   // A4 width  (mm)
static const float PAPER_H = 297.0f;   // A4 height (mm)

// =============================================================================
//  KAHAN COMPENSATED SUMMATION  (Knuth Vol.2 §4.2.2, Algorithm A)
// =============================================================================
struct KahanAccum {
    float sum = 0.0f;
    float c   = 0.0f;

    void reset() { sum = 0.0f; c = 0.0f; }

    inline void add(float x) {
        float y = x - c;
        float t = sum + y;
        c   = (t - sum) - y;
        sum = t;
    }

    void damp(float factor) { sum *= factor; c *= factor; }
};

// =============================================================================
//  GESTURE STATE  (displacement-based)
// =============================================================================
static bool  isDrawing         = false;
static float g_roll0           = 0.0f, g_pitch0  = 0.0f, g_yaw0  = 0.0f;
static float g_prevRoll        = 0.0f, g_prevPitch = 0.0f, g_prevYaw = 0.0f;
static float g_pathLen         = 0.0f;
static int   g_drawSampleCount = 0;
static String mlGesture        = "none";
static int    mlConfidence     = 0;

static inline float angleDiff(float a, float b) {
    float d = fmodf(a - b + 180.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d - 180.0f;
}

// =============================================================================
//  QUATERNION MATHEMATICS
//  Convention: q rotates body frame → world frame.
//  World frame (BNO080 rotation vector ENU): Z = up, XY = horizontal.
// =============================================================================
struct Quat {
    float w, x, y, z;
    Quat() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
};

// q_out = a * b  (Hamilton product)
inline Quat quat_mul(const Quat& a, const Quat& b) {
    return Quat(
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    );
}

// Conjugate = inverse for a unit quaternion
inline Quat quat_conj(const Quat& q) { return Quat(q.w, -q.x, -q.y, -q.z); }

// Normalise (guards against floating-point drift in the IMU output)
inline Quat quat_norm(const Quat& q) {
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-10f) return Quat();
    return Quat(q.w/n, q.x/n, q.y/n, q.z/n);
}

// Rotate vector (vx,vy,vz) by quaternion q → (ox,oy,oz)
// Efficient form of  v' = q ⊗ [0,v] ⊗ q*
//   t = 2*(q_vec × v)
//   v' = v + q_w*t + (q_vec × t)
inline void quat_rotate(const Quat& q,
                        float vx, float vy, float vz,
                        float& ox, float& oy, float& oz)
{
    float tx = 2.0f*(q.y*vz - q.z*vy);
    float ty = 2.0f*(q.z*vx - q.x*vz);
    float tz = 2.0f*(q.x*vy - q.y*vx);
    ox = vx + q.w*tx + (q.y*tz - q.z*ty);
    oy = vy + q.w*ty + (q.z*tx - q.x*tz);
    oz = vz + q.w*tz + (q.x*ty - q.y*tx);
}

// Quaternion → Euler angles (ZYX convention, degrees)
void quat_to_euler_deg(const Quat& q, float& roll, float& pitch, float& yaw) {
    // Roll  (rotation about X)
    float sinr = 2.0f*(q.w*q.x + q.y*q.z);
    float cosr = 1.0f - 2.0f*(q.x*q.x + q.y*q.y);
    roll  = atan2f(sinr, cosr) * 180.0f / (float)M_PI;

    // Pitch (rotation about Y)
    float sinp = 2.0f*(q.w*q.y - q.z*q.x);
    pitch = asinf(fmaxf(-1.0f, fminf(1.0f, sinp))) * 180.0f / (float)M_PI;

    // Yaw   (rotation about Z)
    float siny = 2.0f*(q.w*q.z + q.x*q.y);
    float cosy = 1.0f - 2.0f*(q.y*q.y + q.z*q.z);
    yaw   = atan2f(siny, cosy) * 180.0f / (float)M_PI;
}

// =============================================================================
//  1-D KALMAN FILTER  (smooths projected 2-D coordinates)
// =============================================================================
struct KF1D {
    float pos, vel;
    float P[2][2];
    float Q_pos, Q_vel, R_meas;

    void init(float q_pos, float q_vel, float r_meas) {
        pos = 0.0f;  vel = 0.0f;
        P[0][0] = 100.0f;  P[0][1] = 0.0f;
        P[1][0] = 0.0f;    P[1][1] = 10.0f;
        Q_pos = q_pos;  Q_vel = q_vel;  R_meas = r_meas;
    }

    float update(float z, float dt, bool zero_vel) {
        float pos_p = pos + vel * dt;
        float vel_p = vel;
        float Pp00 = P[0][0] + dt*(P[1][0] + P[0][1]) + dt*dt*P[1][1] + Q_pos;
        float Pp01 = P[0][1] + dt*P[1][1];
        float Pp10 = P[1][0] + dt*P[1][1];
        float Pp11 = P[1][1] + Q_vel;
        float R_eff = R_meas;
        if (zero_vel) { vel_p *= 0.15f; R_eff = R_meas * 0.05f; }
        float S  = Pp00 + R_eff;
        float K0 = Pp00 / S;
        float K1 = Pp10 / S;
        float innov = z - pos_p;
        pos = pos_p + K0 * innov;
        vel = vel_p + K1 * innov;
        P[0][0] = Pp00 - K0 * Pp00;
        P[0][1] = Pp01 - K0 * Pp01;
        P[1][0] = Pp10 - K1 * Pp00;
        P[1][1] = Pp11 - K1 * Pp01;
        return pos;
    }
};

// =============================================================================
//  VEC3 TYPE  (kept for lever-arm cross-products)
// =============================================================================
typedef float Vec3[3];

inline void vec3_cross(Vec3 out, const Vec3 a, const Vec3 b) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

// Returns false if length ~ 0
inline bool vec3_normalize(Vec3 v) {
    float len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (len < 1e-6f) return false;
    v[0] /= len;  v[1] /= len;  v[2] /= len;
    return true;
}

// =============================================================================
//  ORIENTATION + 3-D DEAD-RECKONING STATE
// =============================================================================
Quat q_current;                   // Latest quaternion from BNO080 rotation vector
bool quat_valid = false;          // True once first valid quaternion received

// 3-D velocity + position in world frame (metres, Kahan)
KahanAccum velX, velY, velZ;
KahanAccum posX, posY, posZ;

KF1D kfX, kfY;                   // Kalman filters on projected 2-D result

float currentX = 0.0f, currentY = 0.0f;
float currentRoll = 0.0f, currentPitch = 0.0f, currentYaw = 0.0f;
bool  imuReady    = false;        // True after 'z' is pressed (origin set)
unsigned long lastIMUTime = 0;

// ── Paper plane (set at calibration) ──────────────────────────────────────────
//  BNO080 world frame: Z is up (ENU).  A flat horizontal paper is in XY.
//  paper_x_axis and paper_y_axis are world-frame unit vectors spanning the paper.
//  paper_normal is perpendicular to the paper, pointing away from it.
//  For a horizontal paper these are fixed; for a tilted surface they would be
//  computed from the pen orientation at calibration time.
float paper_x_axis[3] = { 1.0f, 0.0f, 0.0f };  // "right"  on paper in world
float paper_y_axis[3] = { 0.0f, 1.0f, 0.0f };  // "up"     on paper in world
float paper_normal[3] = { 0.0f, 0.0f, 1.0f };  // normal to paper (= world Z up)
float paper_origin[3] = { 0.0f, 0.0f, 0.0f };  // world position at calibration

// Project 3-D world position (relative to paper_origin) onto paper plane → (m, m)
inline void project_to_paper(float wx, float wy, float wz,
                              float& px, float& py)
{
    float rx = wx - paper_origin[0];
    float ry = wy - paper_origin[1];
    float rz = wz - paper_origin[2];
    px = rx*paper_x_axis[0] + ry*paper_x_axis[1] + rz*paper_x_axis[2];
    py = rx*paper_y_axis[0] + ry*paper_y_axis[1] + rz*paper_y_axis[2];
}

// Fixed tilted sheet (goal A): pen points toward/into the paper; outward normal ~ -pen.
// paper_x = "right on page" via Gram-Schmidt from world +X (fallback +Y); paper_y = n x x.
inline void set_paper_basis_from_pen_world(float px, float py, float pz) {
    paper_normal[0] = -px;
    paper_normal[1] = -py;
    paper_normal[2] = -pz;
    if (!vec3_normalize(paper_normal)) {
        paper_normal[0] = 0.0f;  paper_normal[1] = 0.0f;  paper_normal[2] = 1.0f;
    }

    float refx = 1.0f, refy = 0.0f, refz = 0.0f;
    float dot = refx*paper_normal[0] + refy*paper_normal[1] + refz*paper_normal[2];
    float gx = refx - dot * paper_normal[0];
    float gy = refy - dot * paper_normal[1];
    float gz = refz - dot * paper_normal[2];
    float glen2 = gx*gx + gy*gy + gz*gz;
    if (glen2 < 0.01f) {
        refx = 0.0f;  refy = 1.0f;  refz = 0.0f;
        dot = refx*paper_normal[0] + refy*paper_normal[1] + refz*paper_normal[2];
        gx = refx - dot * paper_normal[0];
        gy = refy - dot * paper_normal[1];
        gz = refz - dot * paper_normal[2];
        glen2 = gx*gx + gy*gy + gz*gz;
    }
    if (glen2 < 1e-8f) {
        paper_normal[0] = 0.0f;  paper_normal[1] = 0.0f;  paper_normal[2] = 1.0f;
        paper_x_axis[0] = 1.0f;  paper_x_axis[1] = 0.0f;  paper_x_axis[2] = 0.0f;
        paper_y_axis[0] = 0.0f;  paper_y_axis[1] = 1.0f;  paper_y_axis[2] = 0.0f;
        return;
    }
    float iglen = 1.0f / sqrtf(glen2);
    paper_x_axis[0] = gx * iglen;
    paper_x_axis[1] = gy * iglen;
    paper_x_axis[2] = gz * iglen;

    vec3_cross(paper_y_axis, paper_normal, paper_x_axis);
    if (!vec3_normalize(paper_y_axis)) {
        paper_y_axis[0] = 0.0f;  paper_y_axis[1] = 1.0f;  paper_y_axis[2] = 0.0f;
    }
}

// =============================================================================
//  PEN DIRECTION  (for 3-D visualisation overlay on frontend)
// =============================================================================
static const float PEN_AXIS[3] = { 0.0f, 1.0f, 0.0f }; // pen tip along body Y
float pen_dir_ref[3]           = { 0.0f, 0.0f, 0.0f };
float pen_dir[3]               = { 0.0f, 0.0f, 0.0f };
bool  ref_captured             = false;
float SCALE_X                  = 400.0f;
float SCALE_Y                  = 400.0f;


// ── Raw sensor snapshot for backend integration ───────────────────────────────
float last_ax = 0, last_ay = 0, last_az = 0;
float last_gx = 0, last_gy = 0, last_gz = 0;
// =============================================================================
//  DRIFT SUPPRESSION PARAMETERS
// =============================================================================
static const float ZUPT_LIN_THRESH  = 0.08f;  // m/s²
static const float ZUPT_GYRO_THRESH = 0.03f;  // rad/s
static const float ACCEL_DEADBAND   = 0.05f;  // m/s²
static const float MAX_VEL_MS       = 0.50f;  // m/s  (writing speed limit)
static const float BIAS_ALPHA       = 0.02f;  // EMA weight for bias update
float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;

// =============================================================================
//  LEVER ARM COMPENSATION
//  IMU is between wrist and MCP; pen tip is 12 cm further along body Y.
//  a_pen = a_IMU + (α × r) + (ω × (ω × r))
// =============================================================================
#define LEVER_X_M  0.00f
#define LEVER_Y_M  0.12f
#define LEVER_Z_M  0.00f
static const Vec3 R_LEVER = { LEVER_X_M, LEVER_Y_M, LEVER_Z_M };
float omega_prev[3] = { 0.0f, 0.0f, 0.0f };

// =============================================================================
//  SHAPE DETECTION & TRAJECTORY
// =============================================================================
String currentShape = "none";
#define MAX_POINTS 2000
float trajX[MAX_POINTS], trajY[MAX_POINTS];
int   trajCount = 0;


float calib_pd1[3] = {0,0,0};  // pen_dir at top-left corner
bool  calib_step1_done = false;


void storePoint(float x, float y) {
    if (trajCount < MAX_POINTS) { trajX[trajCount] = x; trajY[trajCount] = y; trajCount++; }
}

bool detectLine() {
    if (trajCount < 10) return false;
    float x1=trajX[0], y1=trajY[0], x2=trajX[trajCount-1], y2=trajY[trajCount-1];
    float A=y2-y1, B=x1-x2, C=x2*y1-x1*y2;
    float denom = sqrtf(A*A + B*B);
    if (denom < 1e-6f) return false;
    float err = 0;
    for (int i = 0; i < trajCount; i++)
        err += fabsf(A*trajX[i] + B*trajY[i] + C) / denom;
    return (err / trajCount) < 5.0f;
}

bool detectCircle() {
    if (trajCount < 20) return false;
    float cx=0, cy=0;
    for (int i = 0; i < trajCount; i++) { cx += trajX[i]; cy += trajY[i]; }
    cx /= trajCount;  cy /= trajCount;
    float r = 0;
    for (int i = 0; i < trajCount; i++)
        r += sqrtf((trajX[i]-cx)*(trajX[i]-cx) + (trajY[i]-cy)*(trajY[i]-cy));
    r /= trajCount;
    float var = 0;
    for (int i = 0; i < trajCount; i++) {
        float d = sqrtf((trajX[i]-cx)*(trajX[i]-cx) + (trajY[i]-cy)*(trajY[i]-cy));
        var += fabsf(d - r);
    }
    return (var / trajCount) < 5.0f;
}

// =============================================================================
//  IMU + PERIPHERALS
// =============================================================================
BNO080 myIMU;

const int ipFlexPin        = 16;
const int mcpFlexPin       = 18;
const int indexFlexPin     = 4;
const int indexMcpFlexPin  = 5;
const int middleFlexPin    = 6;
const int middleMcpFlexPin = 15;

// =============================================================================
//  WIFI / HTTP
// =============================================================================
unsigned long lastSend = 0;
WiFiClient    keepAliveClient;
HTTPClient    http;
bool          httpBegun = false;

inline float finiteOrZero(float x) { return (isnan(x) || isinf(x)) ? 0.0f : x; }

void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500); Serial.print('.'); attempts++;
    }
    if (WiFi.status() == WL_CONNECTED)
        Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
    else {
        Serial.println("\nWiFi FAILED — continuing offline.");
        Serial.printf("Tried SSID: '%s'\n", WIFI_SSID);
    }
}

bool probeTCP(int retries = 5, int delay_ms = 500) {
    for (int i = 0; i < retries; i++) {
        WiFiClient probe;
        Serial.printf("Probing %s:%d (attempt %d)... ", SERVER_IP, SERVER_PORT, i+1);
        if (probe.connect(SERVER_IP, SERVER_PORT)) {
            Serial.println("reachable");
            probe.stop();
            return true;
        }
        Serial.println("not yet");
        probe.stop();
        delay(delay_ms);
    }
    Serial.println("NOT reachable after retries.");
    return false;
}
bool ensureHttpConnection() {
    if (WiFi.status() != WL_CONNECTED) return false;
    if (httpBegun) { http.end(); httpBegun = false; }
    keepAliveClient.stop();
    if (!keepAliveClient.connected()) {
        if (httpBegun) { http.end(); httpBegun = false; }
        if (!keepAliveClient.connect(SERVER_IP, SERVER_PORT)) return false;
        keepAliveClient.setNoDelay(true);
        String url = String("http://") + SERVER_IP + ":" + SERVER_PORT + SERVER_PATH;
        http.setReuse(true);
        http.setConnectTimeout(3000);
        http.setTimeout(5000);
        if (!http.begin(keepAliveClient, url)) { keepAliveClient.stop(); return false; }
        http.addHeader("Content-Type", "application/json");
        http.addHeader("Connection",   "keep-alive");
        httpBegun = true;
    }
    return true;
}

bool postJSON_oneshot(const char* buf, size_t len) {
    HTTPClient one;
    String url = String("http://") + SERVER_IP + ":" + SERVER_PORT + SERVER_PATH;
    one.setConnectTimeout(3000);
    one.setTimeout(5000);
    if (!one.begin(url)) return false;
    one.addHeader("Content-Type", "application/json");
    int code = one.POST((uint8_t*)buf, len);
    one.end();
    return (code > 0 && code < 400);
}

// =============================================================================
//  DISPLACEMENT GESTURE CLASSIFICATION
// =============================================================================
void classifyGesture() {
    if (g_drawSampleCount < 5) {
        Serial.println("[WARN] Too few samples — draw more slowly.");
        mlGesture = "unknown";  mlConfidence = 0;
        return;
    }

    extern float currentRoll, currentPitch, currentYaw;
    float dr   = angleDiff(currentRoll,  g_roll0);
    float dp   = angleDiff(currentPitch, g_pitch0);
    float dy   = angleDiff(currentYaw,   g_yaw0);
    float disp = sqrtf(dr*dr + dp*dp + dy*dy);
    float straight = (g_pathLen > 1e-6f) ? fminf(disp / g_pathLen, 1.0f) : 0.0f;

    Serial.printf("[DISP] samples=%d  displacement=%.2f°  path=%.2f°  straight=%.4f\n",
                  g_drawSampleCount, disp, g_pathLen, straight);

#if USE_DISPLACEMENT
    bool isLine  = (disp >= GESTURE_THRESHOLD);
    float margin = fabsf(disp - GESTURE_THRESHOLD) / fmaxf(GESTURE_THRESHOLD, 1.0f);
    Serial.printf("[DISP] method=DISPLACEMENT  threshold=%.2f°  → %s\n",
                  GESTURE_THRESHOLD, isLine ? "LINE" : "CIRCLE");
#else
    bool isLine  = (straight >= GESTURE_THRESHOLD);
    float denom  = fmaxf(GESTURE_THRESHOLD, 1.0f - GESTURE_THRESHOLD);
    float margin = fabsf(straight - GESTURE_THRESHOLD) / fmaxf(denom, 1e-6f);
    Serial.printf("[DISP] method=STRAIGHTNESS  threshold=%.4f  → %s\n",
                  GESTURE_THRESHOLD, isLine ? "LINE" : "CIRCLE");
#endif

    mlConfidence = (int)fminf(50.0f + margin * 50.0f, 100.0f);
    mlGesture    = isLine ? "line" : "circle";
    Serial.printf("[RESULT] *** %s ***  confidence=%d%%\n",
                  mlGesture.c_str(), mlConfidence);
}

void startGestureRecording() {
    isDrawing         = true;
    g_roll0           = currentRoll;
    g_pitch0          = currentPitch;
    g_yaw0            = currentYaw;
    g_prevRoll        = currentRoll;
    g_prevPitch       = currentPitch;
    g_prevYaw         = currentYaw;
    g_pathLen         = 0.0f;
    g_drawSampleCount = 0;
    mlGesture         = "collecting...";
    mlConfidence      = 0;
    Serial.printf("[REC] Start — R=%.1f P=%.1f Y=%.1f\n", g_roll0, g_pitch0, g_yaw0);
}

// =============================================================================
//  TELEMETRY — SEND JSON PAYLOAD
// =============================================================================
void sendPayloadJSON(
    float roll, float pitch, float yaw,
    int fIP, int fMCP, int fIdx, int fIdxMCP, int fMid, int fMidMCP,
    float x, float y)
{
    if (!ensureHttpConnection()) { Serial.println("HTTP not ready."); return; }

    JsonDocument doc;
    // Euler angles
    doc["roll"]          = finiteOrZero(roll);
    doc["pitch"]         = finiteOrZero(pitch);
    doc["yaw"]           = finiteOrZero(yaw);
    // Flex sensors
    doc["flexIP"]        = fIP;
    doc["flexMCP"]       = fMCP;
    doc["flexIndex"]     = fIdx;
    doc["flexIndexMCP"]  = fIdxMCP;
    doc["flexMiddle"]    = fMid;
    doc["flexMiddleMCP"] = fMidMCP;
    // 2-D projected paper coordinates (mm)
    doc["x"]             = finiteOrZero(x);
    doc["y"]             = finiteOrZero(y);
    // State
    doc["calibrated"]    = imuReady;
    doc["shape"]         = currentShape;
    doc["gesture_ml"]    = mlGesture.c_str();
    doc["confidence"]    = mlConfidence;
    doc["is_drawing"]    = isDrawing;
    // 3-D world position (mm) — for frontend 3-D scene
    doc["pos3d_x"]       = finiteOrZero(posX.sum * M_TO_MM);
    doc["pos3d_y"]       = finiteOrZero(posY.sum * M_TO_MM);
    doc["pos3d_z"]       = finiteOrZero(posZ.sum * M_TO_MM);
    // Quaternion — for frontend 3-D orientation indicator
    doc["q_w"]           = finiteOrZero(q_current.w);
    doc["q_x"]           = finiteOrZero(q_current.x);
    doc["q_y"]           = finiteOrZero(q_current.y);
    doc["q_z"]           = finiteOrZero(q_current.z);
    // Pen direction unit vector (world frame) — for direction-overlay mode
    doc["pdx"]           = finiteOrZero(pen_dir[0]);
    doc["pdy"]           = finiteOrZero(pen_dir[1]);
    doc["pdz"]           = finiteOrZero(pen_dir[2]);
    doc["scale_x"]       = SCALE_X;
    doc["scale_y"]       = SCALE_Y;
    doc["ax"] = finiteOrZero(last_ax);  // raw accel (m/s²), body frame
    doc["ay"] = finiteOrZero(last_ay);
    doc["az"] = finiteOrZero(last_az);
    doc["gx"] = finiteOrZero(last_gx);  // raw gyro (rad/s), body frame
    doc["gy"] = finiteOrZero(last_gy);
    doc["gz"] = finiteOrZero(last_gz);

    char   buf[1280];
    size_t len = serializeJson(doc, buf, sizeof(buf));
    if (len == 0 || len >= sizeof(buf)) { Serial.println("JSON overflow"); return; }

    int code = http.POST((uint8_t*)buf, len);
    if (code > 0) {
        WiFiClient* stream = http.getStreamPtr();
        String resp;
        while (stream->available()) resp += (char)stream->read();
        if (code != 200 || resp.isEmpty() || resp.length() < 2) { http.end(); return; }

        JsonDocument respDoc;
        if (!deserializeJson(respDoc, resp)) {
            const char* cmd = respDoc["command"] | "";

            if (strcmp(cmd, "detectShape") == 0) {
                currentShape = detectCircle() ? "circle" : detectLine() ? "line" : "unknown";
                trajCount    = 0;
                Serial.println("Shape (XY): " + currentShape);
            }
            else if (strcmp(cmd, "resetPos") == 0) {
                velX.reset(); velY.reset(); velZ.reset();
                posX.reset(); posY.reset(); posZ.reset();
                kfX.init(0.5f, 2.0f, 10.0f);
                kfY.init(0.5f, 2.0f, 10.0f);
                currentX = currentY = 0.0f;
                trajCount = 0;
                Serial.println("Position reset via server.");
            }
            else if (strcmp(cmd, "gravityInit") == 0) {
                // Handled immediately in startGravityCalib(); server just triggers it
                extern void startGravityCalib();
                startGravityCalib();
                Serial.println("Calibration triggered by server.");
            }
            else if (strcmp(cmd, "startDrawing") == 0) {
                startGestureRecording();
                Serial.println("Drawing started via server.");
            }
            else if (strcmp(cmd, "stopDrawing") == 0) {
                isDrawing = false;
                classifyGesture();
                Serial.printf("Drawing stopped. Result: %s (%d%%)\n",
                              mlGesture.c_str(), mlConfidence);
            }
            else if (strcmp(cmd, "setScale") == 0) {
                SCALE_X = respDoc["sx"] | SCALE_X;
                SCALE_Y = respDoc["sy"] | SCALE_Y;
            }
        }
    } else {
        Serial.printf("POST failed (%s) — one-shot fallback...\n",
                      http.errorToString(code).c_str());
        keepAliveClient.stop(); http.end(); httpBegun = false;
        if (!postJSON_oneshot(buf, len))
            Serial.println("One-shot also failed.");
        else
            Serial.println("One-shot OK.");
    }
}

// =============================================================================
//  IMU INIT
// =============================================================================
uint8_t scanForBNO080() {
    Serial.println(F("\nScanning I2C..."));
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X\n", addr);
            if (addr == 0x4A || addr == 0x4B) found = addr;
        }
        delay(2);
    }
    return found;
}

bool initIMU_Auto() {
    Wire.begin(8, 9);
    Wire.setClock(100000);
    delay(200);
    uint8_t addr = scanForBNO080();
    if (!addr) return false;
    if (!myIMU.begin(addr, Wire)) {
        uint8_t alt = (addr == 0x4A) ? 0x4B : 0x4A;
        if (!myIMU.begin(alt, Wire)) return false;
    }
    delay(300);
    // ── Three sensor reports at 50 Hz ─────────────────────────────────────────
    // Rotation vector: BNO080 on-chip AHRS quaternion (accel+gyro+mag fusion).
    //   Gives DRIFT-FREE orientation.  Replaces our manual gyro integration.
    myIMU.enableRotationVector(20);   // 20 ms = 50 Hz

    // Raw accelerometer: still needed for position double-integration.
    //   Includes gravity; we subtract it using the quaternion (Step 3 in loop).
    myIMU.enableAccelerometer(20);

    // Raw gyroscope: only used for ZUPT magnitude check; not integrated.
    myIMU.enableGyro(20);

    Wire.setClock(400000);
    Serial.println(F("IMU ready. RotationVector + Accelerometer + Gyro enabled @ 50 Hz."));
    return true;
}

// =============================================================================
//  RESET HELPERS
// =============================================================================
void resetPosition() {
    velX.reset(); velY.reset(); velZ.reset();
    posX.reset(); posY.reset(); posZ.reset();
    biasX = 0.0f; biasY = 0.0f; biasZ = 0.0f;
    omega_prev[0] = omega_prev[1] = omega_prev[2] = 0.0f;
    kfX.init(0.5f, 2.0f, 10.0f);
    kfY.init(0.5f, 2.0f, 10.0f);
    currentX = currentY = 0.0f;
    trajCount = 0;
    // Reset paper origin to current 3-D position (always zero right after reset)
    paper_origin[0] = 0.0f;
    paper_origin[1] = 0.0f;
    paper_origin[2] = 0.0f;
}

void startGravityCalib() {
    // With BNO080 rotation vector, gravity alignment is already handled by the
    // on-chip AHRS.  All we need to do here is:
    //   1. Reset 3-D position to zero (this point becomes the origin).
    //   2. Define the paper coordinate system from the current world orientation.
    //   3. Capture the pen direction reference for the overlay mode.
    resetPosition();
    imuReady = true;

    // ── Paper plane (tilted sheet in world) ─────────────────────────────────
    //  Hold the pen roughly perpendicular to the sheet, tip toward the paper.
    //  Outward normal ~ -pen_dir; in-plane axes from Gram-Schmidt (world +X as
    //  paper "right", then y = n x x). If mirrored, flip sign (+pen for normal)
    //  or swap z/c corner order.
    float pdx, pdy, pdz;
    quat_rotate(q_current, PEN_AXIS[0], PEN_AXIS[1], PEN_AXIS[2], pdx, pdy, pdz);
    set_paper_basis_from_pen_world(pdx, pdy, pdz);

    // ── Pen direction reference for overlay mode ─────────────────────────────
    quat_rotate(q_current,
                PEN_AXIS[0], PEN_AXIS[1], PEN_AXIS[2],
                pen_dir_ref[0], pen_dir_ref[1], pen_dir_ref[2]);
    ref_captured = true;

    Serial.printf("[CALIB] Origin set. Pen dir in world: (%.3f, %.3f, %.3f)\n",
                  pen_dir_ref[0], pen_dir_ref[1], pen_dir_ref[2]);
    Serial.println("[CALIB] Ready — press 's' to start gesture.");
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(400);

    connectWiFi();
    probeTCP();

    if (!initIMU_Auto()) {
        Serial.println(F("FATAL: BNO080 init failed.  Check I2C wiring."));
        while (true) delay(1000);
    }

    q_current = Quat();   // identity until first rotation-vector packet
    resetPosition();

    Serial.println(F(
        "\nSetup complete — QUATERNION + 3-D DEAD-RECKONING + 2-D PROJECTION.\n"
        "  z / c  — set origin + paper plane (point pen onto paper then press)\n"
        "  s      — START gesture recording\n"
        "  e      — END gesture → classifies LINE or CIRCLE\n"
        "  d      — detect shape from stored XY trajectory\n"
        "  r      — reset position (keep orientation)\n"
        "  R      — full reset\n"
    ));
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
    unsigned long now = millis();

    // ── 1. Drain BNO080 FIFO ─────────────────────────────────────────────────
    //  With three reports enabled (rotationVector + accel + gyro at 20 ms each),
    //  dataAvailable() fires ~3× per 20 ms cycle.  Each call processes one SHTP
    //  packet; getQuat*()/getAccel*()/getGyro*() always return the LAST received
    //  value for that sensor type.  dt from millis() correctly handles the
    //  finer timestep without over-integrating.
    while (myIMU.dataAvailable()) {

        // ── a. Read all sensor values (latest received for each type) ─────────
        float qw = myIMU.getQuatReal();
        float qi = myIMU.getQuatI();
        float qj = myIMU.getQuatJ();
        float qk = myIMU.getQuatK();

        float ax = myIMU.getAccelX();
        float ay = myIMU.getAccelY();
        float az = myIMU.getAccelZ();

        float gx = myIMU.getGyroX();
        float gy = myIMU.getGyroY();
        float gz = myIMU.getGyroZ();
        last_ax = ax; last_ay = ay; last_az = az;
        last_gx = gx; last_gy = gy; last_gz = gz;

        // ── b. Update quaternion from rotation vector ─────────────────────────
        //  BNO080 outputs zero quat before AHRS converges.  Magnitude check
        //  ensures we only use a valid, normalised quaternion.
        float qmag = sqrtf(qw*qw + qi*qi + qj*qj + qk*qk);
        if (qmag > 0.5f) {
            q_current  = quat_norm(Quat(qw, qi, qj, qk));
            quat_valid = true;
        }

        if (!quat_valid) continue;   // wait for first good quaternion

        // ── c. Euler angles for gesture accumulation + telemetry ──────────────
        quat_to_euler_deg(q_current, currentRoll, currentPitch, currentYaw);

        // ── d. Pen direction vector in world frame (3-D visualisation) ─────────
        quat_rotate(q_current,
                    PEN_AXIS[0], PEN_AXIS[1], PEN_AXIS[2],
                    pen_dir[0],  pen_dir[1],  pen_dir[2]);

        // ── e. Accumulate gesture path length ─────────────────────────────────
        if (isDrawing) {
            if (g_drawSampleCount > 0) {
                float dr  = angleDiff(currentRoll,  g_prevRoll);
                float dp  = angleDiff(currentPitch, g_prevPitch);
                float dy_ = angleDiff(currentYaw,   g_prevYaw);
                g_pathLen += sqrtf(dr*dr + dp*dp + dy_*dy_);
            }
            g_prevRoll  = currentRoll;
            g_prevPitch = currentPitch;
            g_prevYaw   = currentYaw;
            g_drawSampleCount++;
        }

        // ── f. 3-D dead-reckoning (only after 'z' sets the origin) ────────────
        if (!imuReady) continue;
        if (ax == 0.0f && ay == 0.0f && az == 0.0f) continue; // empty accel packet

        float dt = (now - lastIMUTime) / 1000.0f;
        if (dt <= 0.0f || dt > 0.5f) dt = 1.0f / SEND_HZ;
        lastIMUTime = now;

        // ── Step 1: Lever arm correction (body frame) ─────────────────────────
        //  Compute angular acceleration α = Δω / Δt, then:
        //    a_pen_body = a_IMU_body + tangential(α×r) + centripetal(ω×(ω×r))
        Vec3 omega = { gx, gy, gz };
        Vec3 alpha = {
            (gx - omega_prev[0]) / dt,
            (gy - omega_prev[1]) / dt,
            (gz - omega_prev[2]) / dt
        };
        omega_prev[0] = gx;  omega_prev[1] = gy;  omega_prev[2] = gz;

        Vec3 omega_x_r, centripetal, tangential;
        vec3_cross(omega_x_r,   omega, R_LEVER);
        vec3_cross(centripetal, omega, omega_x_r);
        vec3_cross(tangential,  alpha, R_LEVER);

        float ab_x = ax + centripetal[0] + tangential[0];
        float ab_y = ay + centripetal[1] + tangential[1];
        float ab_z = az + centripetal[2] + tangential[2];

        // ── Step 2: Rotate body-frame accel → world frame via quaternion ───────
        //  a_world = q ⊗ a_body ⊗ q*
        float aw_x, aw_y, aw_z;
        quat_rotate(q_current, ab_x, ab_y, ab_z, aw_x, aw_y, aw_z);

        // ── Step 3: Subtract gravity ───────────────────────────────────────────
        //  BNO080 world frame: Z is UP.  When stationary, aw_z ≈ +G_MS2.
        //  Linear acceleration = a_world − [0, 0, G_MS2].
        float aLx = aw_x;
        float aLy = aw_y;
        float aLz = aw_z - G_MS2;

        // ── Step 4: ZUPT ───────────────────────────────────────────────────────
        float lin_mag  = sqrtf(aLx*aLx + aLy*aLy + aLz*aLz);
        float gyro_mag = sqrtf(gx*gx   + gy*gy   + gz*gz);
        bool  zupt     = (lin_mag < ZUPT_LIN_THRESH) && (gyro_mag < ZUPT_GYRO_THRESH);

        if (zupt) {
            // Slow-EMA bias update (only trust very-still moments)
            biasX = BIAS_ALPHA * aLx + (1.0f - BIAS_ALPHA) * biasX;
            biasY = BIAS_ALPHA * aLy + (1.0f - BIAS_ALPHA) * biasY;
            biasZ = BIAS_ALPHA * aLz + (1.0f - BIAS_ALPHA) * biasZ;
            velX.reset();  velY.reset();  velZ.reset();
        }

        // ── Step 5: Bias correction + dead-band ───────────────────────────────
        float aLx_c = aLx - biasX;
        float aLy_c = aLy - biasY;
        float aLz_c = aLz - biasZ;
        if (fabsf(aLx_c) < ACCEL_DEADBAND) aLx_c = 0.0f;
        if (fabsf(aLy_c) < ACCEL_DEADBAND) aLy_c = 0.0f;
        if (fabsf(aLz_c) < ACCEL_DEADBAND) aLz_c = 0.0f;

        // ── Step 6: Integrate → 3-D velocity (Kahan) ──────────────────────────
        velX.add(aLx_c * dt);
        velY.add(aLy_c * dt);
        velZ.add(aLz_c * dt);

        // Velocity cap (physically impossible to exceed writing speed)
        auto clampVel = [](KahanAccum& v) {
            if (v.sum >  MAX_VEL_MS) v.sum =  MAX_VEL_MS;
            if (v.sum < -MAX_VEL_MS) v.sum = -MAX_VEL_MS;
        };
        clampVel(velX);  clampVel(velY);  clampVel(velZ);

        // ── Step 7: Integrate → 3-D position (Kahan) ──────────────────────────
        posX.add(velX.sum * dt);
        posY.add(velY.sum * dt);
        posZ.add(velZ.sum * dt);

        // ── Step 8: Project 3-D world position → 2-D paper coordinates ─────────
        //  project_to_paper() takes dot products with paper_x_axis / paper_y_axis.
        //  For horizontal paper these equal (posX.sum, posY.sum) directly.
        float proj_x_m, proj_y_m;
        project_to_paper(posX.sum, posY.sum, posZ.sum, proj_x_m, proj_y_m);

        float u_m = proj_x_m;
        float v_m = proj_y_m;
        const float max_x_m = PAPER_W / M_TO_MM;
        const float max_y_m = PAPER_H / M_TO_MM;
        bool wall = false;
        if (u_m < 0.0f) { u_m = 0.0f; wall = true; }
        else if (u_m > max_x_m) { u_m = max_x_m; wall = true; }
        if (v_m < 0.0f) { v_m = 0.0f; wall = true; }
        else if (v_m > max_y_m) { v_m = max_y_m; wall = true; }

        // ── Step 9: Paper boundary wall (works for tilted planes) ─────────────
        if (wall) {
            posX.sum = paper_origin[0] + u_m*paper_x_axis[0] + v_m*paper_y_axis[0];
            posY.sum = paper_origin[1] + u_m*paper_x_axis[1] + v_m*paper_y_axis[1];
            posZ.sum = paper_origin[2] + u_m*paper_x_axis[2] + v_m*paper_y_axis[2];
            posX.c = posY.c = posZ.c = 0.0f;
            velX.reset();  velY.reset();  velZ.reset();
        }

        float rawX_mm = u_m * M_TO_MM;
        float rawY_mm = v_m * M_TO_MM;

        // ── Step 10: Kalman filter on 2-D result ──────────────────────────────
        currentX = kfX.update(rawX_mm, dt, zupt);
        currentY = kfY.update(rawY_mm, dt, zupt);

        storePoint(currentX, currentY);
    }   // end while(dataAvailable)

    // ── 2. Flex sensors ───────────────────────────────────────────────────────
    int fIP     = analogRead(ipFlexPin);
    int fMCP    = analogRead(mcpFlexPin);
    int fIdx    = analogRead(indexFlexPin);
    int fIdxMCP = analogRead(indexMcpFlexPin);
    int fMid    = analogRead(middleFlexPin);
    int fMidMCP = analogRead(middleMcpFlexPin);

    // ── 3. Serial commands ────────────────────────────────────────────────────
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 'z':
                if (!calib_step1_done) {
                    // Record pen direction at corner 1
                    calib_pd1[0] = pen_dir[0];
                    calib_pd1[1] = pen_dir[1];
                    calib_pd1[2] = pen_dir[2];
                    calib_step1_done = true;
                    Serial.println("Top-left recorded. Now aim at BOTTOM-RIGHT and press 'c'.");
                }
                break;
            case 'c':
                if (calib_step1_done) {
                    // pen_dir now points to bottom-right
                    float* pd1 = calib_pd1;
                    float* pd2 = pen_dir;   // current pen direction

                    // Paper X axis = direction from top-left to bottom-right (horizontal component)
                    paper_x_axis[0] = pd2[0] - pd1[0];
                    paper_x_axis[1] = pd2[1] - pd1[1];
                    paper_x_axis[2] = pd2[2] - pd1[2];
                    // Normalize
                    float len = sqrtf(paper_x_axis[0]*paper_x_axis[0] + 
                                    paper_x_axis[1]*paper_x_axis[1] + 
                                    paper_x_axis[2]*paper_x_axis[2]);
                    if (len < 1e-6f) {
                        Serial.println("[CALIB] z/c failed: corners too close.");
                        calib_step1_done = false;
                        break;
                    }
                    paper_x_axis[0]/=len; paper_x_axis[1]/=len; paper_x_axis[2]/=len;

                    // Plane spanned by the two aim rays from the wrist: n || pd1 x pd2
                    Vec3 c1 = { calib_pd1[0], calib_pd1[1], calib_pd1[2] };
                    Vec3 c2 = { pd2[0], pd2[1], pd2[2] };
                    vec3_cross(paper_normal, c1, c2);
                    if (!vec3_normalize(paper_normal)) {
                        Serial.println("[CALIB] z/c failed: corners collinear — try again.");
                        calib_step1_done = false;
                        break;
                    }
                    // In-plane Y: y = n x x  (so x x y = n)
                    vec3_cross(paper_y_axis, paper_normal, paper_x_axis);
                    if (!vec3_normalize(paper_y_axis)) {
                        Serial.println("[CALIB] z/c failed: diagonal ∥ normal.");
                        calib_step1_done = false;
                        break;
                    }

                    vec3_cross(paper_normal, paper_x_axis, paper_y_axis);
                    vec3_normalize(paper_normal);
                    
                    resetPosition();
                    imuReady = true;
                    calib_step1_done = false;
                    Serial.println("Paper plane calibrated from two corners.");
                }
                break;
            case 's':
                startGestureRecording();
                Serial.println("Drawing STARTED — move pen, then press 'e'.");
                break;

            case 'e':
                isDrawing = false;
                Serial.printf("[REC] End — %d samples, path=%.2f°\n",
                              g_drawSampleCount, g_pathLen);
                classifyGesture();
                break;

            case 'd':
                currentShape = detectCircle() ? "circle"
                             : detectLine()   ? "line"
                                              : "unknown";
                Serial.println("Shape (XY): " + currentShape);
                trajCount = 0;
                break;

            case 'r':
                resetPosition();
                Serial.println("Position reset (orientation kept).");
                break;

            case 'R':
                q_current  = Quat();
                quat_valid = false;
                resetPosition();
                imuReady = false;
                Serial.println("Full reset. Press 'z' to set new origin.");
                break;
            

            default: break;
        }
    }

    // ── 4. Telemetry ──────────────────────────────────────────────────────────
    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi dropped — reconnecting...");
            connectWiFi();
            if (httpBegun) { http.end(); httpBegun = false; }
            keepAliveClient.stop();
        } else {
            sendPayloadJSON(
                currentRoll, currentPitch, currentYaw,
                fIP, fMCP, fIdx, fIdxMCP, fMid, fMidMCP,
                currentX, currentY
            );
        }
    }
}