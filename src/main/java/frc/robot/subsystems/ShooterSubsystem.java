//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ICommand;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class ShooterSubsystem extends SubsystemBase implements ISubsystem {
  TalonFX m_leftMotor;
  TalonFX m_rightMotor;
  CANSparkMax m_rotateMotor;
  VoltageOut m_spinVoltageOut = new VoltageOut(0);
  PIDController m_rotatePID = new PIDController(0.01, 0, 0);
  double m_spinSpeed = 0;
  public void updateDashboard() {
    if(this.getCurrentCommand() != null){
      ((ICommand)this.getCurrentCommand()).updateDashboard();
      SmartDashboard.putString("ShooterSubsystem", this.getCurrentCommand().getName());
    }
    SmartDashboard.putNumber("Shooter Angle", getRotateAngle());
    SmartDashboard.putNumber("Shooter Speed", m_spinSpeed);
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    initialize();
  }

  public void initialize() {

    RobotContainer.subsystems.add(this);
    m_leftMotor = new TalonFX(0,k.ROBORIO_CAN_IDS.NAME);
    m_rightMotor = new TalonFX(0,k.ROBORIO_CAN_IDS.NAME);
    m_rotateMotor = new CANSparkMax(0, MotorType.kBrushless);
    
  }
  /** Spin the spinners
   * 
   * @param _speed +/- 1.0
   */
  public void spin(double _speed){
    m_spinSpeed = _speed * k.ROBOT.BATTERY_MAX_VOLTS;
    m_leftMotor.setControl(m_spinVoltageOut.withEnableFOC(true).withOutput(m_spinSpeed));
    m_rightMotor.setControl(m_spinVoltageOut.withEnableFOC(true).withOutput(-m_spinSpeed));
  }
  /** Rotate the shooter to an anlge
   * 
   * @param _angle Degrees
   */
  public void rotate(double _angle){
    double pid = m_rotatePID.calculate(getRotateAngle(), _angle);
    MathUtil.clamp(pid, -2, 2);
    m_rotateMotor.setVoltage(pid*k.ROBOT.BATTERY_MAX_VOLTS);
  }
  public double getRotateAngle(){
    return m_rotateMotor.getEncoder().getPosition() / k.SHOOTER.ROTATE_GEAR_RATIO * 360.0;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
