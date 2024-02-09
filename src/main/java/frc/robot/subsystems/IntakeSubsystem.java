//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ICommand;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;

public class IntakeSubsystem extends SubsystemBase implements ISubsystem {
  TalonFX m_spinMotor;
  VoltageOut m_voltageOutSpin;
  public void updateDashboard() {
    if(this.getCurrentCommand() != null){
      ((ICommand)this.getCurrentCommand()).updateDashboard();
      SmartDashboard.putString("IntakeSubsystem", this.getCurrentCommand().getName());
    }
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    initialize();
  }

  public void initialize() {
    RobotContainer.subsystems.add(this);
    m_spinMotor = new TalonFX(k.ROBORIO_CAN_IDS.INTAKE_SPIN);
    m_voltageOutSpin = new VoltageOut(0);
  }
  public void spin(double _speed){
    m_spinMotor.setControl(m_voltageOutSpin.withEnableFOC(true).withOutput(_speed * k.ROBOT.BATTERY_MAX_VOLTS));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
