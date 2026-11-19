// =============================================================================
//  IMU Pen Tracker v2 — Dead-Reckoning Edition
//  ESP32-S2 + BNO080
//
//  ── ALGORITHM ────────────────────────────────────────────────────────────────
//
//  Step 1  GRAVITY INITIALISATION ('z' command)
//    Hold the pen still for ~0.5 s.  The raw accelerometer output while
//    stationary equals the specific force (reaction to gravity) in the
//    body frame.  We average 25 samples → accel_body_static.
//    We then build a 3×3 rotation matrix R (body → world) such that
//        R * accel_body_static = [0, 0, +g]    (world Z = up)
//    using the analytical solution derived from Gram-Schmidt.
//
//  Step 2  ROTATION MATRIX MAINTENANCE (every IMU tick)
//    The gyroscope gives us angular velocity ω = [gx, gy, gz] rad/s in
//    the body frame.  We propagate the rotation matrix via Rodrigues'
//    exact rotation formula (no small-angle approximation):
//        R_new = R * dR(ω, dt)
//    After each step we re-orthogonalise R with Gram-Schmidt on its columns
//    to prevent floating-point drift from making it non-orthogonal.
//
//  Step 3  WORLD-FRAME LINEAR ACCELERATION
//    Rotate the raw accelerometer vector to world frame, then subtract
//    the known gravity vector:
//        a_lin = R * a_body − [0, 0, +g]
//    We only need the X and Y components (motion in the plane of the paper).
//
//  Step 4  DOUBLE INTEGRATION — KAHAN COMPENSATED SUMMATION
//    Reference: D.E. Knuth, "The Art of Computer Programming",
//               Vol. 2, §4.2.2, Algorithm A (compensated summation).
//    Standard floating-point accumulation of small values introduces
//    O(n·ε) rounding error.  Kahan's algorithm reduces this to O(ε):
//
//        // Instead of: sum += x
//        y    = x − compensation
//        t    = sum + y
//        comp = (t − sum) − y     ← recovers the lost low-order bits
//        sum  = t
//
//    Applied twice per axis:
//        velocity  ← integrate(a_lin, dt)   [Kahan]
//        position  ← integrate(velocity, dt) [Kahan]
//
//  Step 5  ZUPT — Zero Velocity Update
//    If both the linear-accel magnitude and the gyro magnitude fall below
//    their respective thresholds, the pen is stationary.  We:
//      (a) Hard-damp the velocity Kahan accumulator (× 0.1).
//      (b) Pass zero_vel=true to the Kalman filter.
//
//  Step 6  1-D KALMAN FILTER (X and Y independently)
//    State  : [position (mm), velocity (mm/s)]
//    Model  : constant velocity  F = [[1,dt],[0,1]]
//    Observe: position only      H = [1, 0]
//    Acts as a final smoother on top of the integrated position.
//
//  ── COORDINATE OUTPUT ────────────────────────────────────────────────────────
//    currentX, currentY in mm relative to the position at the moment 'z'
//    was pressed.  Positive X = pen's initial right, positive Y = pen's
//    initial forward.
//
//  ── IMPORTANT NOTES ON DRIFT ─────────────────────────────────────────────────
//    Pure inertial dead-reckoning accumulates drift quadratically with time
//    (bias²·t²/2 for position).  ZUPT breaks this accumulation every time
//    the pen lifts off the paper.  For longer strokes, an external position
//    fix (UWB anchor, optical sensor, or periodic paper-edge touch) would
//    be needed for absolute accuracy.
//
//  ── SERIAL COMMANDS ──────────────────────────────────────────────────────────
//    z  — hold pen still → initialise gravity → zero position
//    d  — detect shape from stored trajectory
//    r  — reset position only (keep R; good for new stroke after ZUPT)
//    R  — full reset (R → Identity, re-run gravity init)
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>
// ADD these instead:
#include "gesture_model_data.h"


// ─────────────────────────── WiFi & Server ───────────────────────────────────
#define WIFI_SSID    "iQOO Neo7"
#define WIFI_PASS    "sahil@222"
#define SERVER_IP    "10.230.225.77"
#define SERVER_PORT  5001
#define SERVER_PATH  "/ingest"

// ── ML Gesture Recognition ────────────────────────────────────────────────────
#define NUM_FEATURES    21     // must match Python: X.shape[1]
#define ML_WINDOW_SIZE  50     // samples per inference window
#define ML_STEP_SIZE    25     // run inference every 25 new samples


// ─────────────────────────── Telemetry Rate ──────────────────────────────────
#define SEND_HZ 50
static const uint32_t SEND_INTERVAL_MS = 1000 / SEND_HZ;

// ─────────────────────────── Physical Constants ──────────────────────────────
static const float G_MS2    = 9.81f;         // gravity (m/s²)
static const float M_TO_MM  = 1000.0f;       // metres → millimetres
static const float PAPER_W  = 210.0f;        // A4 width  (mm)
static const float PAPER_H  = 297.0f;        // A4 height (mm)

// =============================================================================
//  KAHAN COMPENSATED SUMMATION
//  Knuth, "The Art of Computer Programming", Vol.2 §4.2.2, Algorithm A.
//
//  Usage:
//    KahanAccum k;
//    k.reset();
//    k.add(x * dt);    // instead of  sum += x * dt
//    float result = k.sum;
// =============================================================================
struct KahanAccum {
    float sum = 0.0f;
    float c   = 0.0f;   // running compensation for lost low-order bits

    void reset() { sum = 0.0f; c = 0.0f; }

    inline void add(float x) {
        float y = x - c;            // compensated value
        float t = sum + y;          // provisional new sum (may lose low bits)
        c   = (t - sum) - y;        // recover the lost bits into c
        sum = t;
    }

    // Multiplicative damping (used by ZUPT to zero velocity without hard reset)
    void damp(float factor) {
        sum *= factor;
        c   *= factor;
    }
};



// Circular sensor buffer
struct SensorSample {
    float roll, pitch, yaw;
    // float flex[3];             // IP, MCP, Index, IndexMCP, Middle, MiddleMCP
};
static SensorSample mlBuf[ML_WINDOW_SIZE];
static int  mlBufHead    = 0;
static int  mlBufCount   = 0;
static int  mlNewSamples = 0;
static bool isDrawing    = false;

#define VOTE_QUEUE_SIZE 8
static int    voteQueue[VOTE_QUEUE_SIZE] = {-1,-1,-1,-1,-1,-1,-1,-1};
static int    voteHead    = 0;
static String mlGesture   = "none";
static int    mlConfidence = 0;

// Returns majority prediction and sets mlConfidence
int tallyVotes() {
    int circles = 0, lines = 0;
    for (int i = 0; i < VOTE_QUEUE_SIZE; i++) {
        if (voteQueue[i] == 1) circles++;
        else if (voteQueue[i] == 0) lines++;
    }
    int total = circles + lines;
    if (total == 0) { mlConfidence = 0; return -1; }
    if (circles >= lines) {
        mlConfidence = (circles * 100) / total;
        return 1;  // circle
    } else {
        mlConfidence = (lines * 100) / total;
        return 0;  // line
    }
}






// =============================================================================
//  1-D KALMAN FILTER
//  State : [pos (mm), vel (mm/s)]
//  Model : constant velocity    F = [[1, dt], [0, 1]]
//  Sensor: position only        H = [1, 0]
//
//  All matrix algebra expanded analytically — no dynamic allocation.
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

    // Predict-update cycle.
    // zero_vel: true when ZUPT detects the pen is stationary.
    float update(float z, float dt, bool zero_vel) {
        // ── Predict ──────────────────────────────────────────────────────────
        float pos_p = pos + vel * dt;
        float vel_p = vel;
        // F·P·Fᵀ  (analytically expanded for F = [[1,dt],[0,1]])
        float Pp00 = P[0][0] + dt*(P[1][0] + P[0][1]) + dt*dt*P[1][1] + Q_pos;
        float Pp01 = P[0][1] + dt*P[1][1];
        float Pp10 = P[1][0] + dt*P[1][1];
        float Pp11 = P[1][1] + Q_vel;
        // ── ZUPT: bleed off velocity and trust the near-zero measurement ──────
        float R_eff = R_meas;
        if (zero_vel) {
            vel_p  *= 0.15f;        // aggressively damp predicted velocity
            R_eff   = R_meas * 0.05f;  // lower R → snap to measurement
        }
        // ── Update ───────────────────────────────────────────────────────────
        float S  = Pp00 + R_eff;
        float K0 = Pp00 / S;   // Kalman gain — position state
        float K1 = Pp10 / S;   // Kalman gain — velocity state
        float innov = z - pos_p;
        pos = pos_p + K0 * innov;
        vel = vel_p + K1 * innov;
        // ── Covariance update  (I − K·H)·P_pred ──────────────────────────────
        P[0][0] = Pp00 - K0 * Pp00;
        P[0][1] = Pp01 - K0 * Pp01;
        P[1][0] = Pp10 - K1 * Pp00;
        P[1][1] = Pp11 - K1 * Pp01;
        return pos;
    }
};

// =============================================================================
//  3×3 ROTATION MATRIX ALGEBRA
//  Convention: v_world = R * v_body
//  Stored row-major: R[row][col]
//  Columns = body-frame basis vectors expressed in world coordinates.
//  Rows    = world-frame basis vectors expressed in body coordinates.
// =============================================================================
typedef float Mat3[3][3];
typedef float Vec3[3];

inline void mat3_identity(Mat3 M) {
    memset(M, 0, sizeof(Mat3));
    M[0][0] = M[1][1] = M[2][2] = 1.0f;
}

// w = M * v
inline void mat3_vec(Vec3 w, const Mat3 M, const Vec3 v) {
    for (int i = 0; i < 3; i++) {
        w[i] = 0;
        for (int j = 0; j < 3; j++) w[i] += M[i][j] * v[j];
    }
}

// out = A * B
void mat3_mul(Mat3 out, const Mat3 A, const Mat3 B) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            out[i][j] = 0;
            for (int k = 0; k < 3; k++) out[i][j] += A[i][k] * B[k][j];
        }
}

// Gram-Schmidt re-orthogonalisation on the COLUMNS of R.
// Prevents floating-point drift from accumulating over thousands of
// Rodrigues updates into a non-orthogonal (volume-distorting) matrix.
void mat3_orthogonalise(Mat3 R) {
    // Extract columns
    float c0[3] = {R[0][0], R[1][0], R[2][0]};
    float c1[3] = {R[0][1], R[1][1], R[2][1]};
    float c2[3];

    // Normalise column 0
    float n0 = sqrtf(c0[0]*c0[0] + c0[1]*c0[1] + c0[2]*c0[2]);
    if (n0 > 1e-10f) { c0[0]/=n0; c0[1]/=n0; c0[2]/=n0; }

    // Orthogonalise column 1 against column 0
    float d01 = c0[0]*c1[0] + c0[1]*c1[1] + c0[2]*c1[2];
    c1[0] -= d01*c0[0]; c1[1] -= d01*c0[1]; c1[2] -= d01*c0[2];
    float n1 = sqrtf(c1[0]*c1[0] + c1[1]*c1[1] + c1[2]*c1[2]);
    if (n1 > 1e-10f) { c1[0]/=n1; c1[1]/=n1; c1[2]/=n1; }

    // Column 2 = column 0 × column 1  (enforces right-handedness)
    c2[0] = c0[1]*c1[2] - c0[2]*c1[1];
    c2[1] = c0[2]*c1[0] - c0[0]*c1[2];
    c2[2] = c0[0]*c1[1] - c0[1]*c1[0];

    // Write back
    R[0][0]=c0[0]; R[1][0]=c0[1]; R[2][0]=c0[2];
    R[0][1]=c1[0]; R[1][1]=c1[1]; R[2][1]=c1[2];
    R[0][2]=c2[0]; R[1][2]=c2[1]; R[2][2]=c2[2];
}

// ── Rodrigues' rotation formula ───────────────────────────────────────────────
// Propagates R for one IMU timestep given body-frame angular velocity ω (rad/s)
// and timestep dt (s).
//
// The incremental rotation dR is computed exactly via Rodrigues:
//     dR = I·cosθ + sin(θ)·K + (1−cosθ)·k·kᵀ
// where θ = |ω|·dt,  k = ω/|ω| (unit rotation axis),  K = skew(k).
//
// Then R_new = R * dR   (right-multiply because ω is in body frame).
// Finally, re-orthogonalise every step to prevent drift.
void rotation_update(Mat3 R, float gx, float gy, float gz, float dt) {
    float theta = sqrtf(gx*gx + gy*gy + gz*gz) * dt;
    if (theta < 1e-10f) return;   // no rotation this tick

    float kx = gx*dt / theta;
    float ky = gy*dt / theta;
    float kz = gz*dt / theta;
    float s = sinf(theta), c = cosf(theta), t = 1.0f - c;

    Mat3 dR;
    dR[0][0] = t*kx*kx + c;       dR[0][1] = t*kx*ky - s*kz;  dR[0][2] = t*kx*kz + s*ky;
    dR[1][0] = t*kx*ky + s*kz;    dR[1][1] = t*ky*ky + c;      dR[1][2] = t*ky*kz - s*kx;
    dR[2][0] = t*kx*kz - s*ky;    dR[2][1] = t*ky*kz + s*kx;   dR[2][2] = t*kz*kz + c;

    Mat3 Rnew;
    mat3_mul(Rnew, R, dR);         // R_new = R * dR  (body-frame update)
    memcpy(R, Rnew, sizeof(Mat3));
    mat3_orthogonalise(R);         // prevent floating-point rot-matrix drift
}

// ── Initialise R from a static gravity measurement ───────────────────────────
// We want R such that:  R * a_body_static = [0, 0, +g]
// This means  row-2 of R = a_body_static / |a_body_static|.
// Rows 0 and 1 are any two vectors orthogonal to row-2 (Gram-Schmidt).
void rotation_init_from_gravity(Mat3 R, float ax, float ay, float az) {
    float g = sqrtf(ax*ax + ay*ay + az*az);
    if (g < 0.5f) {
        Serial.println("WARN: gravity sample too small — defaulting to Identity.");
        mat3_identity(R);
        return;
    }
    // Row 2 = normalised gravity direction in body frame
    float zr[3] = { ax/g, ay/g, az/g };

    // Row 0: start from [1,0,0]; if nearly parallel to row-2, use [0,1,0]
    float xr[3] = { 1.0f, 0.0f, 0.0f };
    if (fabsf(zr[0]) > 0.9f) { xr[0] = 0.0f; xr[1] = 1.0f; }

    // Gram-Schmidt: project xr perpendicular to zr
    float dot = xr[0]*zr[0] + xr[1]*zr[1] + xr[2]*zr[2];
    xr[0] -= dot*zr[0];  xr[1] -= dot*zr[1];  xr[2] -= dot*zr[2];
    float nx = sqrtf(xr[0]*xr[0] + xr[1]*xr[1] + xr[2]*xr[2]);
    xr[0] /= nx;  xr[1] /= nx;  xr[2] /= nx;

    // Row 1 = row-2 × row-0  (right-handed)
    float yr[3] = {
        zr[1]*xr[2] - zr[2]*xr[1],
        zr[2]*xr[0] - zr[0]*xr[2],
        zr[0]*xr[1] - zr[1]*xr[0]
    };

    R[0][0]=xr[0]; R[0][1]=xr[1]; R[0][2]=xr[2];
    R[1][0]=yr[0]; R[1][1]=yr[1]; R[1][2]=yr[2];
    R[2][0]=zr[0]; R[2][1]=zr[1]; R[2][2]=zr[2];
}

// Cross product:  out = a × b
inline void vec3_cross(Vec3 out, const Vec3 a, const Vec3 b) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

// =============================================================================
//  EULER EXTRACTION FROM R  (for telemetry display only, ZYX convention)
//  R = Rz(ψ)·Ry(θ)·Rx(φ)  →  R[2][0] = -sinθ, etc.
// =============================================================================
void mat3_to_euler_deg(const Mat3 R, float &roll, float &pitch, float &yaw) {
    float pitchRad = asinf(fmaxf(-1.0f, fminf(1.0f, -R[2][0])));
    float rollRad  = atan2f(R[2][1], R[2][2]);
    float yawRad   = atan2f(R[1][0], R[0][0]);
    roll  = rollRad  * 180.0f / (float)M_PI;
    pitch = pitchRad * 180.0f / (float)M_PI;
    yaw   = yawRad   * 180.0f / (float)M_PI;
}

// =============================================================================
//  DEAD-RECKONING STATE
// =============================================================================
Mat3 R_mat;          // Body → World rotation matrix

// Kahan integrators for velocity (m/s) and position (m), X and Y axes
KahanAccum velX, velY;
KahanAccum posX, posY;

// Kalman filters for final smoothed X, Y output (in mm)
KF1D kfX, kfY;

float currentX = 0.0f, currentY = 0.0f;  // smoothed output, mm
float currentRoll = 0.0f, currentPitch = 0.0f, currentYaw = 0.0f;  // degrees, from R_mat
bool  imuReady = false;
unsigned long lastIMUTime = 0;

// Gravity calibration (accumulated while 'z' is held)
static const int G_SAMPLE_COUNT = 25;     // ~0.5 s at 50 Hz
float  g_accum[3]  = {0, 0, 0};
int    g_samples   = 0;
bool   g_calibrating = false;

// =============================================================================
//  LEVER ARM COMPENSATION
//  The IMU sits between the wrist and MCP joints — NOT at the pen tip.
//  During writing, the wrist/fingers rotate, causing centripetal and tangential
//  accelerations at the IMU that do NOT correspond to pen-tip displacement.
//
//  Corrected pen-tip acceleration in body frame:
//      a_pen = a_IMU + (α × r) + (ω × (ω × r))
//  where:
//      r     = lever arm from IMU centre to pen tip, in body frame (metres)
//      ω     = gyro reading (rad/s)
//      α     = dω/dt, angular acceleration (rad/s²), estimated each tick
//
//  HOW TO MEASURE r:
//    Hold the pen naturally as if writing.  The body-frame axes depend on
//    how the IMU PCB is oriented on the glove:
//      r_Y  — distance along the finger/pen axis from the IMU to the pen tip
//             (typically 0.08–0.15 m; measure with a ruler)
//      r_X  — lateral offset (usually near 0)
//      r_Z  — vertical offset (usually near 0)
//    Adjust the three defines below, reflash, and compare to a known stroke.
// =============================================================================
#define LEVER_X_M   0.00f   // m — lateral offset   (+ = thumb side)
#define LEVER_Y_M   0.12f   // m — forward to tip   (+ = toward pen tip) ← tune this
#define LEVER_Z_M   0.00f   // m — vertical offset  (+ = dorsal/back of hand)

static const Vec3 R_LEVER = { LEVER_X_M, LEVER_Y_M, LEVER_Z_M };

// Previous gyro sample — needed to compute angular acceleration α = Δω/dt
float omega_prev[3] = { 0.0f, 0.0f, 0.0f };

// ── Drift-suppression constants ───────────────────────────────────────────────
// ZUPT: pen considered stationary when BOTH conditions hold
static const float ZUPT_LIN_THRESH  = 0.08f;   // m/s² — tighter than before
static const float ZUPT_GYRO_THRESH = 0.03f;   // rad/s

// Technique 1 — Dead-band: accelerations this small are sensor noise, ignore them
// A pen resting still should read ~0 linear accel; 0.05 m/s² ≈ 5 mg threshold.
static const float ACCEL_DEADBAND   = 0.05f;   // m/s²

// Technique 2 — Velocity cap: fastest realistic pen stroke < 500 mm/s (0.5 m/s)
static const float MAX_VEL_MS       = 0.50f;   // m/s

// Technique 3 — Bias estimator: slow EMA of residual linear-accel during ZUPT.
// Updated only when stationary; α small so bias adapts slowly (avoids cancelling
// real motion that happens to be slow).
static const float BIAS_ALPHA       = 0.02f;   // EMA rate during ZUPT frames
float biasX = 0.0f, biasY = 0.0f;             // estimated accelerometer bias (m/s²)

// =============================================================================
//  SHAPE DETECTION & TRAJECTORY  (unchanged)
// =============================================================================
String currentShape = "none";
#define MAX_POINTS 2000
float trajX[MAX_POINTS], trajY[MAX_POINTS];
int   trajCount = 0;

void storePoint(float x, float y) {
    if (trajCount < MAX_POINTS) { trajX[trajCount]=x; trajY[trajCount]=y; trajCount++; }
}

bool detectLine() {
    if (trajCount < 10) return false;
    float x1=trajX[0], y1=trajY[0], x2=trajX[trajCount-1], y2=trajY[trajCount-1];
    float A=y2-y1, B=x1-x2, C=x2*y1-x1*y2, denom=sqrtf(A*A+B*B);
    if (denom < 1e-6f) return false;
    float err = 0;
    for (int i=0; i<trajCount; i++) err += fabsf(A*trajX[i]+B*trajY[i]+C)/denom;
    return (err/trajCount) < 5.0f;
}

bool detectCircle() {
    if (trajCount < 20) return false;
    float cx=0, cy=0;
    for (int i=0; i<trajCount; i++) { cx+=trajX[i]; cy+=trajY[i]; }
    cx/=trajCount; cy/=trajCount;
    float r=0;
    for (int i=0; i<trajCount; i++) r+=sqrtf((trajX[i]-cx)*(trajX[i]-cx)+(trajY[i]-cy)*(trajY[i]-cy));
    r/=trajCount;
    float var=0;
    for (int i=0; i<trajCount; i++) {
        float d=sqrtf((trajX[i]-cx)*(trajX[i]-cx)+(trajY[i]-cy)*(trajY[i]-cy));
        var+=fabsf(d-r);
    }
    return (var/trajCount) < 5.0f;
}

// =============================================================================
//  IMU
// =============================================================================
BNO080 myIMU;

// =============================================================================
//  FLEX SENSOR PINS
// =============================================================================
const int ipFlexPin        = 16;
const int mcpFlexPin       = 18;
const int indexFlexPin     = 4;
const int indexMcpFlexPin  = 5;
const int middleFlexPin    = 6;
const int middleMcpFlexPin = 15;

// =============================================================================
//  WIFI / HTTP  (unchanged from v1)
// =============================================================================
unsigned long lastSend = 0;
WiFiClient    keepAliveClient;
HTTPClient    http;
bool          httpBegun = false;

inline float finiteOrZero(float x) { return (isnan(x)||isinf(x)) ? 0.0f : x; }

void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print('.');
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
    } else {
        Serial.println("\nWiFi FAILED — continuing without WiFi.");
        Serial.println("IMU will still work. Check hotspot is on.");
        Serial.printf("Connecting to: '%s'\n", WIFI_SSID);
    }
}
bool probeTCP() {
    WiFiClient probe;
    Serial.printf("Probing %s:%d ... ", SERVER_IP, SERVER_PORT);
    bool ok = probe.connect(SERVER_IP, SERVER_PORT);
    Serial.println(ok ? "reachable" : "NOT reachable");
    probe.stop();
    return ok;
}

bool ensureHttpConnection() {
    if (WiFi.status() != WL_CONNECTED) return false;
    if (!keepAliveClient.connected()) {
        if (httpBegun) { http.end(); httpBegun = false; }
        if (!keepAliveClient.connect(SERVER_IP, SERVER_PORT)) return false;
        keepAliveClient.setNoDelay(true);
        String url = String("http://") + SERVER_IP + ":" + SERVER_PORT + SERVER_PATH;
        http.setReuse(true); http.setConnectTimeout(3000); http.setTimeout(5000);
        if (!http.begin(keepAliveClient, url)) { keepAliveClient.stop(); return false; }
        http.addHeader("Content-Type",  "application/json");
        http.addHeader("Connection",    "keep-alive");
        httpBegun = true;
    }
    return true;
}

bool postJSON_oneshot(const char* buf, size_t len) {
    HTTPClient one;
    String url = String("http://") + SERVER_IP + ":" + SERVER_PORT + SERVER_PATH;
    one.setConnectTimeout(3000); one.setTimeout(5000);
    if (!one.begin(url)) return false;
    one.addHeader("Content-Type", "application/json");
    int code = one.POST((uint8_t*)buf, len);
    one.end();
    return (code > 0 && code < 400);
}

void sendPayloadJSON(
    float roll, float pitch, float yaw,
    int fIP, int fMCP, int fIdx, int fIdxMCP, int fMid, int fMidMCP,
    float x, float y)
{
    if (!ensureHttpConnection()) { Serial.println("HTTP not ready."); return; }

    StaticJsonDocument<256> doc;
    doc["roll"]          = finiteOrZero(roll);
    doc["pitch"]         = finiteOrZero(pitch);
    doc["yaw"]           = finiteOrZero(yaw);
    doc["flexIP"]        = fIP;
    doc["flexMCP"]       = fMCP;
    doc["flexIndex"]     = fIdx;
    doc["flexIndexMCP"]  = fIdxMCP;
    doc["flexMiddle"]    = fMid;
    doc["flexMiddleMCP"] = fMidMCP;
    doc["x"]             = finiteOrZero(x);
    doc["y"]             = finiteOrZero(y);
    doc["calibrated"]    = imuReady;
    doc["shape"]         = currentShape;
    doc["gesture_ml"]  = mlGesture.c_str();
    doc["confidence"]  = mlConfidence;
    doc["is_drawing"]  = isDrawing;

    char   buf[384];
    size_t len = serializeJson(doc, buf, sizeof(buf));
    if (len == 0 || len >= sizeof(buf)) { Serial.println("JSON error"); return; }

    int code = http.POST((uint8_t*)buf, len);
    if (code > 0) {
        // ── Parse server command ──────────────────────────────────────────────
        WiFiClient* stream = http.getStreamPtr();
        String resp;
        while (stream->available()) resp += (char)stream->read();
        if (code != 200 || resp.isEmpty() || resp.length() < 2) {
            http.end();
            return;
        }
        StaticJsonDocument<256> respDoc;
        if (!deserializeJson(respDoc, resp)) {
            const char* cmd = respDoc["command"] | "";
            if (strcmp(cmd, "detectShape") == 0) {
                currentShape = detectCircle() ? "circle"
                             : detectLine()   ? "line"
                                              : "unknown";
                trajCount = 0;
                Serial.println("Shape: " + currentShape);
            }
            else if (strcmp(cmd, "resetPos") == 0) {
                // Reset position only — keep orientation
                velX.reset(); velY.reset();
                posX.reset(); posY.reset();
                kfX.init(0.5f, 2.0f, 10.0f);
                kfY.init(0.5f, 2.0f, 10.0f);
                currentX = currentY = 0;
                trajCount = 0;
                Serial.println("Position reset via server.");
            }
            else if (strcmp(cmd, "gravityInit") == 0) {
                // Server-triggered gravity reinit
                g_accum[0] = g_accum[1] = g_accum[2] = 0;
                g_samples    = 0;
                g_calibrating = true;
                imuReady      = false;
                Serial.println("Gravity init triggered by server. Hold still...");
            }
            else if (strcmp(cmd, "startDrawing") == 0) {
                isDrawing = true;
                mlBufHead = 0; mlBufCount = 0; mlNewSamples = 0;
                mlGesture = "collecting..."; mlConfidence = 0;
                for (int i=0; i<VOTE_QUEUE_SIZE; i++) voteQueue[i] = -1;
                Serial.println("Drawing started via server button.");
            }
            else if (strcmp(cmd, "stopDrawing") == 0) {
                isDrawing = false;
                Serial.printf("Drawing stopped via server. Final: %s (%d%%)\n",
                              mlGesture.c_str(), mlConfidence);
            }
        }
    } else {
        // ── HTTP-level failure: teardown keep-alive, retry one-shot ──────────
        Serial.printf("POST failed (%s) — trying one-shot...\n",
                      http.errorToString(code).c_str());
        keepAliveClient.stop(); http.end(); httpBegun = false;
        if (!postJSON_oneshot(buf, len)) Serial.println("One-shot also failed.");
        else                              Serial.println("One-shot succeeded.");
    }
}

// =============================================================================
//  IMU INIT  — enable RAW Accelerometer + Gyro (not Rotation Vector)
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

    // ── Enable Accelerometer + Gyro — NOT the rotation vector ────────────────
    // Accelerometer: calibrated, includes gravity, body frame, m/s²
    // Gyro         : calibrated, angular velocity, body frame, rad/s
    // Both at 20 ms → 50 Hz to match the telemetry rate.
    myIMU.enableAccelerometer(20);
    myIMU.enableGyro(20);

    Wire.setClock(400000);
    Serial.println(F("IMU ready. Using Accelerometer + Gyro (dead-reckoning mode)."));
    return true;
}

// =============================================================================
//  POSITION / VELOCITY RESET HELPERS
// =============================================================================
void resetPosition() {
    velX.reset(); velY.reset();
    posX.reset(); posY.reset();
    biasX = 0.0f; biasY = 0.0f;
    omega_prev[0] = omega_prev[1] = omega_prev[2] = 0.0f;  // avoid alpha spike on restart
    kfX.init(0.5f, 2.0f, 10.0f);
    kfY.init(0.5f, 2.0f, 10.0f);
    currentX = currentY = 0.0f;
    trajCount = 0;
}

void startGravityCalib() {
    g_accum[0] = g_accum[1] = g_accum[2] = 0.0f;
    g_samples     = 0;
    g_calibrating = true;
    imuReady      = false;
    Serial.println("Hold pen STILL for ~0.5 s to calibrate gravity...");
}



// ── Feature extraction — mirrors Python extract_features() exactly ────────────
// ── Helper functions for ML feature extraction ────────────────────────────
// Change buf_median — add 'static' to the tmp array
static float buf_median(float* src, int n) {
    static float tmp[ML_WINDOW_SIZE];   // ← static: lives in RAM, not stack
    for (int i = 0; i < n; i++) tmp[i] = src[i];
    for (int i = 1; i < n; i++) {
        float key = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = key;
    }
    return (n & 1) ? tmp[n/2] : (tmp[n/2-1] + tmp[n/2]) * 0.5f;
}

static float upper_mean(float* data, int n, float mn) {
    float sum = 0.0f; int cnt = 0;
    for (int i = 0; i < n; i++) if (data[i] > mn) { sum += data[i]; cnt++; }
    return cnt > 0 ? sum / cnt : mn;
}

static float ml_zcr(float* data, int n) {
    if (n < 2) return 0.0f;
    float mn = 0.0f;
    for (int i = 0; i < n; i++) mn += data[i];
    mn /= n;
    int crossings = 0;
    float prev = data[0] - mn;
    for (int i = 1; i < n; i++) {
        float cur = data[i] - mn;
        if (prev * cur < 0.0f) crossings++;
        prev = cur;
    }
    return (float)crossings / (n - 1);
}

static float ml_pearson(float* a, float* b, int n) {
    float ma = 0, mb = 0;
    for (int i = 0; i < n; i++) { ma += a[i]; mb += b[i]; }
    ma /= n; mb /= n;
    float num = 0, da2 = 0, db2 = 0;
    for (int i = 0; i < n; i++) {
        float da = a[i]-ma, db = b[i]-mb;
        num += da*db; da2 += da*da; db2 += db*db;
    }
    float denom = sqrtf(da2*db2);
    return denom < 1e-6f ? 0.0f : num/denom;
}



void runMLInference() {
    if (mlBufCount < ML_WINDOW_SIZE) return;

    const int N = ML_WINDOW_SIZE;

    // ── Step 1: Copy window into float arrays ─────────────────────────────
    float rollA[N], pitchA[N], yawA[N];
    for (int i = 0; i < N; i++) {
        int idx   = (mlBufHead - N + i + ML_WINDOW_SIZE) % ML_WINDOW_SIZE;
        rollA[i]  = mlBuf[idx].roll;
        pitchA[i] = mlBuf[idx].pitch;
        yawA[i]   = mlBuf[idx].yaw;
    }

    // ── Step 2: Per-window normalisation (must match training notebook) ────
    float channels[3][N];
    float* src[3] = { rollA, pitchA, yawA };
    for (int c = 0; c < 3; c++) {
        float mn = 0.0f;
        for (int i = 0; i < N; i++) mn += src[c][i];
        mn /= N;
        float var = 0.0f;
        for (int i = 0; i < N; i++) { float e = src[c][i]-mn; var += e*e; }
        float sd = sqrtf(var / N) + 1e-6f;
        for (int i = 0; i < N; i++) channels[c][i] = (src[c][i] - mn) / sd;
    }

    // ── Step 3: Flatten normalised window to 1-D input array ──────────────
    float input[150];   // 50 timesteps × 3 channels
    for (int i = 0; i < N; i++) {
        input[i * 3 + 0] = channels[0][i];   // Roll
        input[i * 3 + 1] = channels[1][i];   // Pitch
        input[i * 3 + 2] = channels[2][i];   // Yaw
    }

    // ── Step 4: Run forward pass ───────────────────────────────────────────
    float prob_circle = gesture_cnn_predict(input);   // no library needed
    int   pred        = (prob_circle > 0.5f) ? 1 : 0;

    Serial.printf("[CNN] circle_prob=%.2f → %s\n",
                  prob_circle, pred == 1 ? "circle" : "line");

    // ── Step 5: Vote queue (same as before) ───────────────────────────────
    voteQueue[voteHead % VOTE_QUEUE_SIZE] = pred;
    voteHead++;

    int v0 = 0, v1 = 0;
    int filled = min(voteHead, VOTE_QUEUE_SIZE);
    for (int i = 0; i < filled; i++) {
        if      (voteQueue[i] == 0) v0++;
        else if (voteQueue[i] == 1) v1++;
    }
    int total = v0 + v1;
    if (total == 0 || filled < 3) return;

    if (v1 >= v0) {
        mlGesture    = "circle";
        mlConfidence = (v1 * 100) / total;
    } else {
        mlGesture    = "line";
        mlConfidence = (v0 * 100) / total;
    }
    Serial.printf("[ML] %s  confidence=%d%%\n", mlGesture.c_str(), mlConfidence);
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
        Serial.println(F("FATAL: BNO080 init failed."));
        while (true) delay(1000);
    }
    
    mat3_identity(R_mat);
    resetPosition();

    Serial.println(F(
        "\nSetup complete — DEAD-RECKONING mode.\n"
        "  z  — hold still, initialise gravity, zero position\n"
        "  d  — detect shape from current trajectory\n"
        "  r  — reset position (keep orientation)\n"
        "  R  — full reset (orientation + position)\n"
    ));
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
    unsigned long now = millis();

    // ── 1. Drain the entire BNO080 FIFO each loop iteration ──────────────────
    // With two sensors (accel + gyro) at 50 Hz, ~100 packets/s arrive.
    // We drain all of them to use the most recent values this tick.
    while (myIMU.dataAvailable()) {
        // Latest values after this packet is parsed
        float ax = myIMU.getAccelX();    // m/s²
        float ay = myIMU.getAccelY();
        float az = myIMU.getAccelZ();
        float gx = myIMU.getGyroX();     // rad/s
        float gy = myIMU.getGyroY();
        float gz = myIMU.getGyroZ();

        // ── Gravity calibration accumulation ─────────────────────────────────
        if (g_calibrating && (ax != 0 || ay != 0 || az != 0)) {
            g_accum[0] += ax; g_accum[1] += ay; g_accum[2] += az;
            g_samples++;
            if (g_samples >= G_SAMPLE_COUNT) {
                g_accum[0] /= g_samples;
                g_accum[1] /= g_samples;
                g_accum[2] /= g_samples;
                rotation_init_from_gravity(R_mat, g_accum[0], g_accum[1], g_accum[2]);
                resetPosition();
                lastIMUTime   = millis();
                imuReady      = true;
                g_calibrating = false;
                Serial.printf("Gravity calib done: g_body=(%.3f, %.3f, %.3f) m/s²\n",
                              g_accum[0], g_accum[1], g_accum[2]);
                Serial.println("IMU ready — start tracing!");
            }
            continue;   // Don't integrate during calibration
        }

        // ── Skip packets with no data ─────────────────────────────────────────
        if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) continue;

        float dt = (now - lastIMUTime) / 1000.0f;
        if (dt <= 0.0f || dt > 0.5f) dt = 1.0f / SEND_HZ;
        lastIMUTime = now;

        // ── Step 2: Update rotation matrix via Rodrigues (ALWAYS) ────────────
        // This runs regardless of imuReady so that roll/pitch/yaw are always
        // live on the website — even before the 'z' gravity calibration.
        rotation_update(R_mat, gx, gy, gz, dt);

        // ── Always extract Euler angles so the website sees them immediately ──
        mat3_to_euler_deg(R_mat, currentRoll, currentPitch, currentYaw);

        // Uncomment to watch orientation live in the Serial Monitor:
        // Serial.printf("R:%.2f  P:%.2f  Y:%.2f  | X:%.1fmm  Y:%.1fmm\n",
        //               currentRoll, currentPitch, currentYaw, currentX, currentY);

        // ── Position integration only after gravity calibration ('z') ─────────
        if (!imuReady) continue;

        // ── Step 3a: Lever arm correction ─────────────────────────────────────
        // The IMU is NOT at the pen tip. Wrist/finger rotation creates
        // centripetal and tangential accelerations at the IMU sensor that
        // do not correspond to actual pen-tip movement on paper.
        //
        // We correct the body-frame acceleration to what the pen tip experiences:
        //     a_pen = a_IMU  +  (α × r)          ← tangential (Euler) term
        //                    +  ω × (ω × r)       ← centripetal term
        //
        // where r = LEVER ARM from IMU to pen tip in body frame.

        Vec3 omega     = { gx, gy, gz };

        // Angular acceleration α ≈ Δω / dt
        Vec3 alpha = {
            (gx - omega_prev[0]) / dt,
            (gy - omega_prev[1]) / dt,
            (gz - omega_prev[2]) / dt
        };
        omega_prev[0] = gx;  omega_prev[1] = gy;  omega_prev[2] = gz;

        // ω × r
        Vec3 omega_cross_r;
        vec3_cross(omega_cross_r, omega, R_LEVER);

        // ω × (ω × r)  — centripetal contribution
        Vec3 centripetal;
        vec3_cross(centripetal, omega, omega_cross_r);

        // α × r  — tangential (Euler) contribution
        Vec3 tangential;
        vec3_cross(tangential, alpha, R_LEVER);

        // Body-frame acceleration corrected to pen-tip location
        Vec3 a_body = {
            ax + centripetal[0] + tangential[0],
            ay + centripetal[1] + tangential[1],
            az + centripetal[2] + tangential[2]
        };

        // ── Step 3b: Rotate corrected acceleration to WORLD frame ─────────────
        Vec3 a_world;
        mat3_vec(a_world, R_mat, a_body);
        // Remove gravity to get true linear acceleration in world frame
        float aLx = a_world[0];
        float aLy = a_world[1];
        // (aLz = a_world[2] - G_MS2 unused for 2-D pen tracking)

        // ── Step 4: ZUPT detection ────────────────────────────────────────────
        float lin_mag  = sqrtf(aLx*aLx + aLy*aLy);
        float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
        bool  zupt     = (lin_mag < ZUPT_LIN_THRESH) && (gyro_mag < ZUPT_GYRO_THRESH);

        if (zupt) {
            // ── Technique 2 (bias estimation) ────────────────────────────────
            // When stationary, the residual linear acceleration IS the bias.
            // Slow EMA so we only track the DC component, not real motion.
            biasX = BIAS_ALPHA * aLx + (1.0f - BIAS_ALPHA) * biasX;
            biasY = BIAS_ALPHA * aLy + (1.0f - BIAS_ALPHA) * biasY;

            // ── Hard-zero velocity (not just damp) ───────────────────────────
            // The previous 10% damp still allowed velocity to bleed through
            // across many ZUPT frames, causing the drift seen in the screenshot.
            velX.reset();
            velY.reset();
        }

        // ── Subtract estimated bias before integrating ────────────────────────
        float aLx_c = aLx - biasX;
        float aLy_c = aLy - biasY;

        // ── Technique 1 (dead-band) ───────────────────────────────────────────
        // Accelerations this small are indistinguishable from sensor noise.
        // Zero them so they never accumulate into a velocity drift.
        if (fabsf(aLx_c) < ACCEL_DEADBAND) aLx_c = 0.0f;
        if (fabsf(aLy_c) < ACCEL_DEADBAND) aLy_c = 0.0f;

        // ── Step 5: KAHAN double-integration (Knuth §4.2.2) ──────────────────
        velX.add(aLx_c * dt);
        velY.add(aLy_c * dt);

        // ── Technique 2 (velocity cap) ────────────────────────────────────────
        // A pen physically cannot move faster than ~500 mm/s on paper.
        // Clamp here so a single bad accel spike cannot explode the velocity.
        if (velX.sum >  MAX_VEL_MS) velX.sum =  MAX_VEL_MS;
        if (velX.sum < -MAX_VEL_MS) velX.sum = -MAX_VEL_MS;
        if (velY.sum >  MAX_VEL_MS) velY.sum =  MAX_VEL_MS;
        if (velY.sum < -MAX_VEL_MS) velY.sum = -MAX_VEL_MS;

        posX.add(velX.sum * dt);
        posY.add(velY.sum * dt);

        // ── Technique 3 (paper boundary wall) ────────────────────────────────
        // If the integrated position tries to leave the paper, clamp it AND
        // zero the velocity in that direction so it can't drift further off-page.
        float rawX_mm = posX.sum * M_TO_MM;
        float rawY_mm = posY.sum * M_TO_MM;

        if (rawX_mm < 0.0f) {
            posX.sum = 0.0f; posX.c = 0.0f; velX.reset(); rawX_mm = 0.0f;
        } else if (rawX_mm > PAPER_W) {
            posX.sum = PAPER_W / M_TO_MM; posX.c = 0.0f; velX.reset(); rawX_mm = PAPER_W;
        }

        if (rawY_mm < 0.0f) {
            posY.sum = 0.0f; posY.c = 0.0f; velY.reset(); rawY_mm = 0.0f;
        } else if (rawY_mm > PAPER_H) {
            posY.sum = PAPER_H / M_TO_MM; posY.c = 0.0f; velY.reset(); rawY_mm = PAPER_H;
        }

        // ── Step 6: Kalman filter on position (mm) ────────────────────────────
        currentX = kfX.update(rawX_mm, dt, zupt);
        currentY = kfY.update(rawY_mm, dt, zupt);

        storePoint(currentX, currentY);
    }

    // ── 2. Flex sensors ───────────────────────────────────────────────────────
    int fIP        = analogRead(ipFlexPin);
    int fMCP       = analogRead(mcpFlexPin);
    int fIdx       = analogRead(indexFlexPin);
    int fIdxMCP    = analogRead(indexMcpFlexPin);
    int fMid       = analogRead(middleFlexPin);
    int fMidMCP    = analogRead(middleMcpFlexPin);
    // ── ML: push sample into buffer when drawing is active ────────────────────
    if (isDrawing) {
        mlBuf[mlBufHead] = {
            currentRoll, currentPitch, currentYaw
        };
        mlBufHead = (mlBufHead + 1) % ML_WINDOW_SIZE;
        if (mlBufCount < ML_WINDOW_SIZE) mlBufCount++;
        mlNewSamples++;

        // Run inference every ML_STEP_SIZE new samples once buffer is full
        if (mlBufCount >= ML_WINDOW_SIZE && mlNewSamples >= ML_STEP_SIZE) {
            mlNewSamples = 0;
            runMLInference();
        }
    }

    // ── 3. Serial commands (one place only) ───────────────────────────────────
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 'z':
                startGravityCalib();
                break;
            case 'd':
                currentShape = detectCircle() ? "circle"
                             : detectLine()   ? "line"
                                              : "unknown";
                Serial.println("Shape: " + currentShape);
                trajCount = 0;
                break;
            case 'r':
                resetPosition();
                Serial.println("Position reset (orientation kept).");
                break;
            case 'R':
                mat3_identity(R_mat);
                resetPosition();
                imuReady = false;
                Serial.println("Full reset. Send 'z' to re-initialise.");
                break;
            case 's':
                // Start Drawing — clear buffer and begin predicting
                isDrawing    = true;
                mlBufHead    = 0;
                mlBufCount   = 0;
                mlNewSamples = 0;
                mlGesture    = "collecting...";
                mlConfidence = 0;
                for (int i=0; i<VOTE_QUEUE_SIZE; i++) voteQueue[i] = -1;
                Serial.println("Drawing STARTED — predicting continuously...");
                break;

            case 'e':
                // End Drawing — freeze the last prediction
                isDrawing = false;
                Serial.printf("Drawing ENDED — final: %s (%d%%)\n",
                              mlGesture.c_str(), mlConfidence);
                break;
            default:
                break;
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
            // currentRoll/Pitch/Yaw are globals updated every IMU tick above
            sendPayloadJSON(
                currentRoll, currentPitch, currentYaw,
                fIP, fMCP, fIdx, fIdxMCP, fMid, fMidMCP,
                currentX, currentY
            );
        }
    }
}