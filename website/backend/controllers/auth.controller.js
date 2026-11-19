import User from "../models/User.js";
import jwt from "jsonwebtoken";

export const signup = async (req, res) => {
  try {
    const { fullName, email, password } = req.body;

    console.log('🔍 Signup attempt for:', email);

    // Validation
    if (!fullName || !email || !password) {
      return res.status(400).json({
        success: false,
        message: "All fields are required"
      });
    }

    if (password.length < 6) {
      return res.status(400).json({
        success: false,
        message: "Password must be at least 6 characters long"
      });
    }

    // Check if user already exists
    const existingUser = await User.findOne({ email });
    if (existingUser) {
      return res.status(400).json({
        success: false,
        message: "User already exists with this email"
      });
    }

    console.log('✅ Creating new user...');

    // Create new user
    const user = new User({
      fullName,
      email,
      password
    });

    await user.save();
    console.log('✅ User saved successfully');

    // Generate JWT token
    const token = jwt.sign(
      { userId: user._id },
      process.env.JWT_SECRET_KEY,
      { expiresIn: '7d' }
    );

    console.log('✅ JWT token generated');

    // Set token in HTTP-only cookie
    res.cookie('jwt', token, {
      httpOnly: true,
      secure: process.env.NODE_ENV === 'production',
      sameSite: 'strict',
      maxAge: 7 * 24 * 60 * 60 * 1000 // 7 days
    });

    // Return success response with token
    res.status(201).json({
      success: true,
      message: "User created successfully",
      user: {
        id: user._id,
        email: user.email,
        fullName: user.fullName
      },
      token: token
    });

    console.log('✅ Signup successful for:', email);
  } catch (error) {
    console.error('❌ Signup error:', error);
    res.status(500).json({
      success: false,
      message: "Signup failed",
      error: error.message
    });
  }
};

export const login = async (req, res) => {
  try {
    const { email, password } = req.body;

    console.log('🔍 Login attempt for email:', email);
    console.log('🔍 Request body:', req.body);

    // Validation
    if (!email || !password) {
      console.log('❌ Missing email or password');
      return res.status(400).json({
        success: false,
        message: "Email and password are required"
      });
    }

    // Find user by email
    const user = await User.findOne({ email });
    if (!user) {
      console.log('❌ User not found for email:', email);
      return res.status(401).json({
        success: false,
        message: "Invalid email or password"
      });
    }

    console.log('✅ User found:', user.email);
    console.log('🔍 Stored password hash:', user.password.substring(0, 20) + '...');

    // Check password
    try {
      const isPasswordValid = await user.comparePassword(password);
      console.log('🔍 Password validation result:', isPasswordValid);
      
      if (!isPasswordValid) {
        console.log('❌ Invalid password for user:', email);
        return res.status(401).json({
          success: false,
          message: "Invalid email or password"
        });
      }
    } catch (passwordError) {
      console.error('❌ Password comparison error:', passwordError);
      return res.status(500).json({
        success: false,
        message: "Authentication error"
      });
    }

    console.log('✅ Password validated successfully');

    // Generate JWT token
    const token = jwt.sign(
      { userId: user._id },
      process.env.JWT_SECRET_KEY,
      { expiresIn: '7d' }
    );

    console.log('✅ JWT token generated');

    // Set token in HTTP-only cookie
    res.cookie('jwt', token, {
      httpOnly: true,
      secure: process.env.NODE_ENV === 'production',
      sameSite: 'strict',
      maxAge: 7 * 24 * 60 * 60 * 1000 // 7 days
    });

    // Return success response with token
    res.json({
      success: true,
      message: "Login successful",
      user: {
        id: user._id,
        email: user.email,
        fullName: user.fullName
      },
      token: token
    });

    console.log('✅ Login successful for user:', email);
  } catch (error) {
    console.error('❌ Login error:', error);
    res.status(500).json({
      success: false,
      message: "Login failed",
      error: error.message
    });
  }
};

export const logout = async (req, res) => {
  try {
    res.clearCookie("jwt", {
      httpOnly: true,
      sameSite: "strict",
      secure: process.env.NODE_ENV === "production"
    });
    
    res.status(200).json({
      success: true,
      message: "Logged out successfully"
    });
  } catch (error) {
    console.error("Error in logout:", error);
    return res.status(500).json({
      success: false,
      message: "Internal server error"
    });
  }
};

export async function getMe(req, res) {
  try {
    const userResponse = req.user.getPublicProfile();
    
    res.status(200).json({
      success: true,
      user: userResponse
    });
  } catch (error) {
    console.error("Error in getMe:", error);
    return res.status(500).json({
      success: false,
      message: "Internal server error"
    });
  }
}

export async function updateProfile(req, res) {
  try {
    const userId = req.user._id;
    const { fullName, bio, phone, dateOfBirth, avatar } = req.body;

    // Validation
    if (!fullName) {
      return res.status(400).json({
        success: false,
        message: "Full name is required"
      });
    }

    // Update user
    const updatedUser = await User.findByIdAndUpdate(
      userId,
      {
        fullName,
        bio,
        phone,
        dateOfBirth,
        avatar
      },
      { new: true, runValidators: true }
    );

    if (!updatedUser) {
      return res.status(404).json({
        success: false,
        message: "User not found"
      });
    }

    const userResponse = updatedUser.getPublicProfile();
    
    res.status(200).json({
      success: true,
      message: "Profile updated successfully",
      user: userResponse
    });

  } catch (error) {
    console.error("Error in updateProfile:", error);
    return res.status(500).json({
      success: false,
      message: "Internal server error"
    });
  }
}

export const getProfile = async (req, res) => {
  try {
    res.json({
      success: true,
      user: {
        id: req.user._id,
        email: req.user.email,
        fullName: req.user.fullName
      }
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      message: "Failed to get profile",
      error: error.message
    });
  }
};


