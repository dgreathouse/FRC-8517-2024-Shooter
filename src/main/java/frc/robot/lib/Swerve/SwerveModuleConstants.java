//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

public class SwerveModuleConstants {
    public String m_name ="";
    /** CAN ID of the drive motor */
    public int m_driveMotorId = 0;
    /** CAN ID of the steer motor */
    public int m_steerMotorId = 0;
    /** CAN ID of the CANcoder used for azimuth */
    public int m_CANcoderId = 0;
    /** Offset of the CANcoder in degrees */
    public double m_CANcoderOffset_deg = 0;
     /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the X axis of the robot
     */
    public double m_locationX_m = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the Y axis of the robot
     */
    public double m_locationY_m = 0;

    /** True if the motors are reversed  */
    public boolean m_isSteerMotorReversed = false;
    public boolean m_isDriveMotorReversed = false;
    public SwerveModuleConstants(){

    }
    public SwerveModuleConstants(String _name, 
        int _driveMotorID, boolean _isDriveReversed, 
        int _steerMotorID, boolean _steerMotorReveresed, 
        int _canCoderID, double _canCoderOffset, 
        double _locationX, double _locationY){
            m_name = _name;
            m_driveMotorId = _driveMotorID;
            m_isDriveMotorReversed = _isDriveReversed;
            m_steerMotorId = _steerMotorID;
            m_isSteerMotorReversed = _steerMotorReveresed;
            m_CANcoderId = _canCoderID;
            m_CANcoderOffset_deg = _canCoderOffset;
            m_locationX_m = _locationX;
            m_locationY_m = _locationY;
    }
 
}
