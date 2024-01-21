//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ICommand;
import frc.robot.lib.ISubsystem;

public class IntakeSubsystem extends SubsystemBase implements ISubsystem {
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
  }
  // public void setTestVoltage(double _volts){
    
  // }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
