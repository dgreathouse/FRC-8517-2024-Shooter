//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandGroups.AutoDoNothing;



import frc.robot.lib.GD;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.LEDs;
import frc.robot.lib.k;



/**
 * In command-based projects, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Subsystems, commands, and trigger mappings should be defined here.
 * 
 */
public class RobotContainer {
  public static Set<ISubsystem> subsystems = new HashSet<>();




  public static final LEDs m_leds = new LEDs(1);

  private Notifier m_telemetry;
  PhotonCamera camera = new PhotonCamera("photonvision");
  
  //public static final CommandPS5Controller s_driverController = new CommandPS5Controller(k.OI.DRIVER_CONTROLLER_PORT);
  public static final CommandPS5Controller s_operatorController = new CommandPS5Controller(k.OI.OPERATOR_CONTROLLER_PORT);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  private void updateDashboard(){
    SmartDashboard.putString("RobotMode", GD.G_RobotMode.toString());
    Iterator<ISubsystem> it = subsystems.iterator();
    while(it.hasNext()){
      it.next().updateDashboard(); // Comment this line out if you want ALL smartdashboard data to be stopped.
    }
  }
  /** This is the constructor for the class. */
  public RobotContainer() {



    camera.setDriverMode(true);
    // Configure the trigger bindings
    configureBindings();

    // Add all autonomous command groups to the list on the Smartdashboard
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    SmartDashboard.putData("Autonomous Play",autoChooser);

    // Setup the dashboard notifier that runs at a slower rate than our main robot periodic.
    m_telemetry = new Notifier(this::updateDashboard);
    m_telemetry.startPeriodic(0.1);

    // Configure the trigger bindings
    configureBindings();
  }
 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   */
  private void configureBindings() {
    m_leds.setAllianceColor();
    //new Trigger(RobotContainer.m_drivetrainSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));
    // s_driverController.square().onTrue(new InstantCommand(m_drivetrainSubsystem::changeDriveMode, m_drivetrainSubsystem));
    // s_driverController.triangle().onTrue(new InstantCommand(m_drivetrainSubsystem::resetYaw, m_drivetrainSubsystem));
  }
  /**
   * @return the command to run in autonomous routine
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
