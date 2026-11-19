import Session from '../models/Session.js';
import Telemetry from '../models/Telemetry.js';

// Start a new recording session
export const startSession = async (req, res) => {
  try {
    const { name, description } = req.body;
    
    const session = new Session({
      name: name || `Session ${new Date().toLocaleString()}`,
      description: description || 'Sensor data recording session',
      startTime: new Date(),
      status: 'recording'
    });

    await session.save();
    
    console.log(`✅ Started recording session: ${session.name}`);
    
    res.json({
      success: true,
      session: {
        id: session._id,
        name: session.name,
        description: session.description,
        startTime: session.startTime,
        status: session.status
      }
    });
  } catch (error) {
    console.error('❌ Error starting session:', error);
    res.status(500).json({
      success: false,
      error: error.message
    });
  }
};

// Stop a recording session
export const stopSession = async (req, res) => {
  try {
    const { sessionId } = req.params;
    
    const session = await Session.findById(sessionId);
    if (!session) {
      return res.status(404).json({
        success: false,
        error: 'Session not found'
      });
    }

    session.endTime = new Date();
    session.duration = session.endTime - session.startTime;
    session.status = 'stopped';
    
    // Count data points
    const dataPoints = await Telemetry.countDocuments({ sessionId });
    session.dataPoints = dataPoints;
    
    await session.save();
    
    console.log(`✅ Stopped recording session: ${session.name} (${dataPoints} data points)`);
    
    res.json({
      success: true,
      session: {
        id: session._id,
        name: session.name,
        description: session.description,
        startTime: session.startTime,
        endTime: session.endTime,
        duration: session.duration,
        dataPoints: session.dataPoints,
        status: session.status
      }
    });
  } catch (error) {
    console.error('❌ Error stopping session:', error);
    res.status(500).json({
      success: false,
      error: error.message
    });
  }
};

// Get all sessions
export const getSessions = async (req, res) => {
  try {
    const sessions = await Session.find()
      .sort({ startTime: -1 })
      .limit(50);
    
    res.json({
      success: true,
      sessions: sessions.map(session => ({
        id: session._id,
        name: session.name,
        description: session.description,
        startTime: session.startTime,
        endTime: session.endTime,
        duration: session.duration,
        dataPoints: session.dataPoints,
        status: session.status,
        formattedDuration: session.formattedDuration
      }))
    });
  } catch (error) {
    console.error('❌ Error getting sessions:', error);
    res.status(500).json({
      success: false,
      error: error.message
    });
  }
};

// Get session data for CSV export
export const getSessionData = async (req, res) => {
  try {
    const { sessionId } = req.params;
    
    const session = await Session.findById(sessionId);
    if (!session) {
      return res.status(404).json({
        success: false,
        error: 'Session not found'
      });
    }

    const telemetryData = await Telemetry.find({ sessionId })
      .sort({ timestamp: 1 });

    res.json({
      success: true,
      session: {
        id: session._id,
        name: session.name,
        description: session.description,
        startTime: session.startTime,
        endTime: session.endTime,
        duration: session.duration,
        dataPoints: session.dataPoints,
        status: session.status
      },
      data: telemetryData.map(record => ({
        timestamp: record.timestamp,
        roll: record.roll,
        pitch: record.pitch,
        yaw: record.yaw,
        flexIP: record.flexIP,
        flexMCP: record.flexMCP,
        flexIndex: record.flexIndex,
        flexIndexMCP: record.flexIndexMCP,
        flexMiddle: record.flexMiddle,
        flexMiddleMCP: record.flexMiddleMCP
      }))
    });
  } catch (error) {
    console.error('❌ Error getting session data:', error);
    res.status(500).json({
      success: false,
      error: error.message
    });
  }
};
