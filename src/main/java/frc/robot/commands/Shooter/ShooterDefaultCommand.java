//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.ICommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterDefaultCommand extends Command implements ICommand {
  ShooterSubsystem m_shooter;
  /** Creates a new ShooterDefaultCommand. */
  public ShooterDefaultCommand(ShooterSubsystem _subsystem) {
    m_shooter = _subsystem;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shooter Set Speed", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.s_operatorController.circle().getAsBoolean()){
      double speed = SmartDashboard.getNumber("Shooter Set Speed", 0);
      m_shooter.spin(speed);
    }else {
      m_shooter.spin(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  @Override
  public void updateDashboard() {

  }
}
