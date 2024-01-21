//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class k {
  public static class CANIVORE_IDS {
    public static final String NAME = "CANivore";
    public static final int PIGEON2_CANID = 5;
    // public static final int DRIVE_LEFT_CANID = 12;
    // public static final int DRIVE_RIGHT_CANID = 13;
    // public static final int DRIVE_BACK_CANID = 11;
    
     
    // public static final int STEER_LEFT_CANID = 22;
    // public static final int STEER_RIGHT_CANID = 23;
    // public static final int STEER_BACK_CANID = 21;

    // public static final int CANCODER_LEFT_CANID = 3;
    // public static final int CANCODER_RIGHT_CANID = 2;
    // public static final int CANCODER_BACK_CANID = 1;
  }
  public static class ROBORIO_CAN_IDS{
    public static final String NAME = "rio";
  }
  public static class CONVERT{
    public static final double DEGREES_TO_RADIANS = 0.017453292519943295;
    public static final double RADIANS_TO_DEGREES = 57.29577951308232;
  }
  public static class ROBOT {
    public static final double PERIOD = 0.02;
    
    
    public static final double BATTERY_MAX_VOLTS = 12.0;


    
    public static final int PD_CANID = 1;  // Power Distribution, Rev or CTRE
    
  }
  public static class OI {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
  public static class DRIVEBASE {
    public static final double WHEEL_BASE_Y_m = 0.47738;
    public static final double WHEEL_BASE_X_m = 0.47851;
    private static final double WHEEL_BASE_XY_AVG_m = (WHEEL_BASE_Y_m + WHEEL_BASE_X_m)/2.0;
    private static final double WHEEL_BASE_CIRCUMFERENCE_m = Math.PI * WHEEL_BASE_XY_AVG_m;
    private static final double WHEEL_BASE_MeterPerRad = WHEEL_BASE_CIRCUMFERENCE_m/(2* Math.PI);
    public static final double TURN_KP = 5.0;
    public static final double TURN_KI = 1.0;
    public static final double TURN_KD = 0.0;
  }
  public static class DRIVE {
    public static final String T_DRIVER_MODE = "DriveMode";
    private static final double MOTOR_PINION_TEETH = 10.0;
    private static final double GEAR_1_TEETH = 34.0;
    private static final double GEAR_2_DRIVE_TEETH = 26.0;
    private static final double GEAR_2_DRIVEN_TEETH = 20.0;
    private static final double GEAR_BEVEL_DRIVE_TEETH = 15.0 ;
    private static final double GEAR_BEVEL_DRIVEN_TEETH = 45.0;

    public static final double GEAR_RATIO = (GEAR_1_TEETH/MOTOR_PINION_TEETH) * (GEAR_2_DRIVEN_TEETH / GEAR_2_DRIVE_TEETH) * (GEAR_BEVEL_DRIVEN_TEETH / GEAR_BEVEL_DRIVE_TEETH);
    public static final double WHEEL_DIAMETER_m = 0.10287;
    private static final double WHEEL_CIRCUMFERENCE_m = Math.PI * WHEEL_DIAMETER_m;
    public static final double WHEEL_RotPerMeter = GEAR_RATIO / WHEEL_CIRCUMFERENCE_m;
    private static final double MOTOR_MAX_VELOCITY_RotPerMin = 6380.0;
    private static final double MOTOR_MAX_VELOCITY_RotPerSec = MOTOR_MAX_VELOCITY_RotPerMin / 60.0;
    private static final double WHEEL_MAX_VELOCITY_RotPerSec = MOTOR_MAX_VELOCITY_RotPerSec / GEAR_RATIO;
    private static final double MOTOR_PEAK_EFFICIENCY_percent = 85.0;
    public static final double MAX_VELOCITY_MeterPerSec = WHEEL_CIRCUMFERENCE_m * WHEEL_MAX_VELOCITY_RotPerSec * MOTOR_PEAK_EFFICIENCY_percent / 100.0;
    public static final double MAX_ANGULAR_VELOCITY_RadianPerSec = MAX_VELOCITY_MeterPerSec * (1/DRIVEBASE.WHEEL_BASE_MeterPerRad);
    public static final double PID_Kp = 0.125;
    public static final double PID_Ki = 0.1;
    public static final double PID_Kv = k.ROBOT.BATTERY_MAX_VOLTS/k.DRIVE.MAX_VELOCITY_MeterPerSec;
    public static final double PID_Ks = 0.18;
  }
  public static class STEER {
    private static final double MOTOR_PINION_TEETH = 8.0;
    private static final double MOTOR_DRIVE_GEAR_TEETH = 24.0;
    private static final double GEAR_1_DRIVE_TEETH = 14.0;
    private static final double GEAR_1_DRIVEN_TEETH = 72.0;
    private static final double CANCODER_GEAR_RATIO = 1.0;
    public static final double GEAR_RATIO = 1/((MOTOR_PINION_TEETH/MOTOR_DRIVE_GEAR_TEETH)*(GEAR_1_DRIVE_TEETH/GEAR_1_DRIVEN_TEETH));
    public static final double GEAR_RATIO_TO_CANCODER = GEAR_RATIO * CANCODER_GEAR_RATIO;
    public static final double PID_Kp = 0.05;
    public static final double PID_Ki = 0.01;
    public static final double PID_MaxV = 0.0;
    public static final double PID_MaxA = 0.0;

  }
  public static class SHOOTER {
    public static final double ROTATE_GEAR_RATIO = 3*4*5;
    private static final double ROTATE_MOTOR_CNT_PER_REV = 42;
    private static final double ROTATE_SHAFT_CNTS_PER_REV = ROTATE_MOTOR_CNT_PER_REV * ROTATE_GEAR_RATIO;
    public static final double ROTATE_CNTS_PER_DEG = ROTATE_SHAFT_CNTS_PER_REV / 360;
    

  }

}
