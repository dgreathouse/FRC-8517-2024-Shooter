![](src/main/java/frc/robot/lib/MythDigLogo1.0.svg)

## Overview
This code is a remake of the CTRE swerve module code. The code has been integrated into the command based programming framework.
This remake drastically changes the CTRE swerve code. Most importantly the PID loops for the Drive and Steer were taken out of the Motor controllers.
All PIDs are calcualted in Degrees->Volts and MPS->Volts with a Feedforward term. CTRE changes were not updated soon enough to test the code before kickoff.
Since time was limited the appoach was to use CTRE VoltageOut mode with the above PIDs and Feedforward values. 

## Robot 
The robot used for this code is our typical 3 wheel swerve drive using Swerve Xs from WCProducts.net. All motors are Falcon FXs with CANCoders.
A CANivore is used with a PRO license to get FOC out of the motors.

## Change for your use
To use this code you must understand the code and adjust all changes for your system. 
Changes needed that I understand at this point are:
- Add another motor for typical 4 wheel swerve. We are one of the only foolish teams to do 3 wheel swerve.
- Change your drive base length and width.
- Tune your PID variables.
- Change all your CANIDs.
- Change gear ratios
- Debug what I have not tested yet...

## Subsystems
- **Drivetrain**
  The drivetrain is our typical swerve drive system that utilizes WCProduts.net swerve X modules. Our drivetrain utilized 3 modules instead of the typical 3.
  The chassis, or frame, this year is a new approach utilizing 3 inch angle aluminum that is 3/16 inch thick. This is a high speed game and impacts are expected.
  The choosen frame strucutre will not bend like our typical 2040 structure that has bent before. 
- **Intake**
  The intake is planned to be a rotating wheeled intake that rotates out over the bumpers picks up Notes from the floor only.
  After picking up the note from the floor the Note is immediatley deliverd to the shooter or the Tramper.
- **Shooter**
  The shooter is a wheeled mechanism that launches the Note at varied angles to allow the robot to score from different distances
- **Climber**
  The climber is a two hook solution that raises the robot of the ground high enough for the Tramper to score in the Trap.
- **Tramper**
  The tramper is responsible for getting a game piece from the source area of from the intake. It is utilized to score in the Amp or Trap.

## Vision - Camera
Vision utilizes 1 or 2 cameras for vision object detection of AprilTags and Notes. Another camera is utilized for the driver on the Smartdashboard since there are some blind spots.
Object detection of a note is mainly planned to be used for autonomous to grab the pixel at center field. AprilTag detection can be used to line up with the Speaker and determine distance to speaker.
With angle and distance the robot can be automated to angle the shooter, adjuste the shooter speed and angle the robot for a speakers shot.

## Autonomous
- Our basic drive strategy is to drive at a angle and velocity in Angle based field centric mode. This allows us to drive in any angle and have the robot PID to an angle. This prevents us from using a PID loop to drive to distance. Since the drive is in velociy mode the time we use is actual distance.

## Pull/Push Requests
Please add a Pull Request for changes that you would like to see.
If you have code fixes or addition you want in, please make a push request with the changes.
More people working on code makes it better if we coordinate and agree on changes/additions.
There are lots of ways to solve a problem when coding, so please understand if changes are not accepted.

## Command Based Programming
The Command based approach used for this code is to use a default command that is associated with the subsystem. The schedular will call the execute() method in the command and from there the subsystem methods are called to drive the motors or whatever you want. 

The other approach for this is where you control all your motors in the subsystem perodic() methods. Commands use the execute method to set variables in the subsystem which the subsystem periodic will act on.

Many ways to do programming, we are accustomed to using the first approach because it seems a little easier to manage but may have more commands sometimes. Pick your approach and deal with it.
