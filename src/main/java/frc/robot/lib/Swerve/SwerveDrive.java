//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.k;

public class SwerveDrive {
    private int m_moduleCount;
    private SwerveModule[] m_modules;
    private Pigeon2 m_pigeon2;
    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;
    private SwerveModulePosition[] m_modulePositions;
    private Translation2d[] m_moduleLocations;
    private OdometryThread m_odometryThread;
    private Field2d m_field;
    private PIDController m_turnPid;



    /* Perform swerve module updates in a separate thread to minimize latency */
    private class OdometryThread extends Thread {
        private BaseStatusSignal[] m_allSignals;
        public int SuccessfulDaqs = 0;
        public int FailedDaqs = 0;
        
        private LinearFilter lowpass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        private double averageLoopTime = 0;

        public OdometryThread() {
            super();
            
            // 4 signals for each module + 2 for Pigeon2
            m_allSignals = new BaseStatusSignal[(m_moduleCount * 4) + 2];
            for (int i = 0; i < m_moduleCount; ++i) {
                var signals = m_modules[i].getSignals();
                m_allSignals[(i * 4) + 0] = signals[0];
                m_allSignals[(i * 4) + 1] = signals[1];
                m_allSignals[(i * 4) + 2] = signals[2];
                m_allSignals[(i * 4) + 3] = signals[3];
            }
            m_allSignals[m_allSignals.length - 2] = m_pigeon2.getYaw();
            m_allSignals[m_allSignals.length - 1] = m_pigeon2.getAngularVelocityZDevice();
        }

        @Override
        public void run() {
            /* Make sure all signals update at around 250hz */
            for (var sig : m_allSignals) {
                sig.setUpdateFrequency(250);
            }
            /* Run as fast as possible, our signals will control the timing */
            while (true) {
                /* Synchronously wait for all signals in drivetrain */
                var status = BaseStatusSignal.waitForAll(0.1, m_allSignals);
                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();
                averageLoopTime = lowpass.calculate(currentTime - lastTime);

                /* Get status of the waitForAll */
                if (status.isOK()) {
                    SuccessfulDaqs++;
                } else {
                    FailedDaqs++;
                }

                /* Now update odometry */
                for (int i = 0; i < m_moduleCount; ++i) {
                    /*
                     * No need to refresh since it's automatically refreshed from the waitForAll()
                     */
                    m_modulePositions[i] = m_modules[i].getPosition(false);
                }
                // Assume Pigeon2 is flat-and-level so latency compensation can be performed
                double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
                        m_pigeon2.getYaw(), m_pigeon2.getAngularVelocityZDevice());

                m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);
                m_field.setRobotPose(m_odometry.getPoseMeters());
            }
        }

        public double getTime() {
            return averageLoopTime;
        }

        public int getSuccessfulDaqs() {
            return SuccessfulDaqs;
        }

        public int getFailedDaqs() {
            return FailedDaqs;
        }
    }
   
    public SwerveDrive() {
        //SwerveModuleConstantsCreator m_constantsCreator = new SwerveModuleConstantsCreator();
        SwerveModuleConstants m_frontRight = new SwerveModuleConstants( 
            "fr",
            13, true,
            23, true, 
            3, 0.0368,
            k.DRIVEBASE.WHEEL_BASE_X_m / 2.0, -k.DRIVEBASE.WHEEL_BASE_Y_m / 2.0);

        SwerveModuleConstants m_frontLeft = new SwerveModuleConstants(
            "fl",
            12, false,
            22, true,  
            2, 0.167, 
            k.DRIVEBASE.WHEEL_BASE_X_m / 2.0, k.DRIVEBASE.WHEEL_BASE_Y_m / 2.0);
        SwerveModuleConstants m_back = new SwerveModuleConstants(
            "b",
            11, false,  
            21, true, 
            1, 0.3185, 
            -k.DRIVEBASE.WHEEL_BASE_X_m / 2.0, 0.0);

        initialize(m_frontLeft, m_frontRight, m_back);    
    }

    public void initialize(SwerveModuleConstants... _modules){
        m_moduleCount = _modules.length;
        m_pigeon2 = new Pigeon2(k.CANIVORE_IDS.PIGEON2_CANID, k.CANIVORE_IDS.NAME);
        m_modules = new SwerveModule[m_moduleCount];
        m_modulePositions = new SwerveModulePosition[m_moduleCount];
        m_moduleLocations = new Translation2d[m_moduleCount];
 
        for(int i = 0; i < m_moduleCount; i++)  {
            m_modules[i] = new SwerveModule(_modules[i]);
            m_moduleLocations[i] = new Translation2d(_modules[i].m_locationX_m, _modules[i].m_locationY_m);
            m_modulePositions[i] = m_modules[i].getPosition(true);
        }

        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);
        m_odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon2.getRotation2d(), getSwervePositions());
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);

        m_turnPid = new PIDController(k.DRIVEBASE.TURN_KP, k.DRIVEBASE.TURN_KI, k.DRIVEBASE.TURN_KD);
        m_turnPid.enableContinuousInput(-Math.PI, Math.PI);
        m_turnPid.setTolerance(Math.toRadians(1));
        
        m_odometryThread = new OdometryThread();
        m_odometryThread.start();
    }
    private SwerveModulePosition[] getSwervePositions() {
        return m_modulePositions;
    }

    public void driveRobotCentric(ChassisSpeeds _speeds) {
        var swerveStates = m_kinematics.toSwerveModuleStates(_speeds);
        for (int i = 0; i < m_moduleCount; ++i) {
            m_modules[i].setDesiredState(swerveStates[i]);
        }
    }

    public void driveFieldCentric(ChassisSpeeds _speeds) {
        var roboCentric = ChassisSpeeds.fromFieldRelativeSpeeds(_speeds, m_pigeon2.getRotation2d());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        for (int i = 0; i < m_moduleCount; ++i) {
            m_modules[i].setDesiredState(swerveStates[i]);
        }
    }

    public void driveAngleFieldCentric(double _xSpeeds, double _ySpeeds, Rotation2d _targetAngle) {
        var currentAngle = m_pigeon2.getRotation2d();
        double rotationalSpeed = m_turnPid.calculate(currentAngle.getRadians(), _targetAngle.getRadians());

        var roboCentric = ChassisSpeeds.fromFieldRelativeSpeeds(
                _xSpeeds, _ySpeeds, rotationalSpeed, m_pigeon2.getRotation2d());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        for (int i = 0; i < m_moduleCount; ++i) {
            m_modules[i].setDesiredState(swerveStates[i]);
        }
    }

    public void driveStopMotion() {
        /* Point every module toward (0,0) to make it close to a X configuration */
        for (int i = 0; i < m_moduleCount; ++i) {
            var angle = m_moduleLocations[i].getAngle();
            m_modules[i].setDesiredState(new SwerveModuleState(0, angle));
        }
    }

    public void resetYaw() {
        m_pigeon2.setYaw(0);
    }

    public Pose2d getPoseMeters() {
        return m_odometry.getPoseMeters();
    }

    public double getSuccessfulDaqs() {
        return m_odometryThread.SuccessfulDaqs;
    }

    public double getFailedDaqs() {
        return m_odometryThread.FailedDaqs;
    }

    public double getRobotYaw() {
        return m_pigeon2.getYaw().getValueAsDouble();
    }

    public boolean isTurnPIDatSetpoint() {
        return m_turnPid.atSetpoint();
    }
         /* Put smartdashboard calls in separate thread to reduce performance impact */
    public void updateDashboard() {
        SmartDashboard.putNumber("Successful Daqs", m_odometryThread.getSuccessfulDaqs());
        SmartDashboard.putNumber("Failed Daqs", m_odometryThread.getFailedDaqs());
        SmartDashboard.putNumber("X Pos", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y Pos", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Angle", m_odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Odometry Loop Time", m_odometryThread.getTime());
        for (int i = 0; i < m_moduleCount; ++i) {
            m_modules[i].updateDashboard();
        }
    }
}
