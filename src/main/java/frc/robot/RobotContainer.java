// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShoot1;
import frc.robot.commands.AutoShoot2;
import frc.robot.commands.AutoShoot2Other;
import frc.robot.commands.ControlWithJoystick;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ElevatorControl;
import frc.robot.commands.FeedBall;
import frc.robot.commands.IntakeArmsDown;
import frc.robot.commands.IntakeArmsUp;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShootBall1;
import frc.robot.commands.ShootBall2;
import frc.robot.commands.ShootBall3;
import frc.robot.commands.ShootBallAuto;
import frc.robot.commands.SpitOutBall;
import frc.robot.commands.Test;
import frc.robot.commands.TrackTarget;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArms;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // subsystems
  private final DriveTrain driveTrain;
  private final Elevator elevator;
  private final Intake intake;
  private final Shooter shooter;
  private final IntakeArms intakearms;
  private final Camera camera;

  //commands
  private final DriveWithJoysticks driveWithJoystick;
  private final ControlWithJoystick controlWithJoystick;
  private final IntakeBall intakeball;
  private final SpitOutBall spitoutball;
  private final FeedBall feedball;
  private final ShootBall1 shootball1;
  private final ShootBall2 shootball2;
  private final ShootBall3 shootball3;
  private final ShootBallAuto shootballAuto;
  private final AutoShoot1 autoshoot1;
  private final AutoShoot2 autoshoot2;
  private final IntakeArmsUp intakeArmsUp;
  private final IntakeArmsDown intakeArmsDown;
  private final TrackTarget trackTarget;
  private final Test test;
  private final ElevatorControl elevatorControl;
  private final AutoIntake autoIntake;
  private final AutoShoot2Other autoShoot2Other;

  //objects
  public static Joystick driverJoystick;
  public static Joystick manipulatorJoystick;
  SendableChooser<String> AutoChooser;
  static SendableChooser<Boolean> timerChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // init subsystems
    driveTrain = new DriveTrain();
    elevator = new Elevator();
    intake = new Intake();
    shooter = new Shooter();
    intakearms = new IntakeArms();
    camera = new Camera();
    // SmartDashboard.putData(driveTrain);
    // SmartDashboard.putData(elevator);
    // SmartDashboard.putData(intake);
    // SmartDashboard.putData(shooter);
    // SmartDashboard.putData(intakearms);
    // SmartDashboard.putData(camera);

    // init objects
    driverJoystick = new Joystick(Constants.DriverJoystickID);
    manipulatorJoystick = new Joystick(Constants.ManipulatorJoystickID);

    // init commands
    driveWithJoystick = new DriveWithJoysticks(driveTrain);
    driveWithJoystick.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoystick);
    controlWithJoystick = new ControlWithJoystick(camera);
    controlWithJoystick.addRequirements(camera);
    camera.setDefaultCommand(controlWithJoystick);
    intakeball = new IntakeBall(intake);
    intakeball.addRequirements(intake);
    spitoutball = new SpitOutBall(intake);
    spitoutball.addRequirements(intake);
    shootball1 = new ShootBall1(shooter);
    shootball1.addRequirements(shooter);
    shootball2 = new ShootBall2(shooter);
    shootball2.addRequirements(shooter);
    shootball3 = new ShootBall3(shooter);
    shootball3.addRequirements(shooter);
    shootballAuto = new ShootBallAuto(shooter);
    shootballAuto.addRequirements(shooter);
    feedball = new FeedBall(intake);
    feedball.addRequirements(intake);
    autoshoot1 = new AutoShoot1(shooter, intake, camera);
    autoshoot1.addRequirements(shooter, intake, camera);
    autoshoot2 = new AutoShoot2(shooter, intake, camera, driveTrain);
    autoshoot2.addRequirements(shooter, intake, camera, driveTrain);
    intakeArmsDown = new IntakeArmsDown(intakearms);
    intakeArmsDown.addRequirements(intakearms);
    intakeArmsUp = new IntakeArmsUp(intakearms);
    intakeArmsUp.addRequirements(intakearms);
    trackTarget = new TrackTarget(camera);
    trackTarget.addRequirements(camera);
    test = new Test(driveTrain);
    test.addRequirements(driveTrain);
    elevatorControl = new ElevatorControl(elevator);
    elevatorControl.addRequirements(elevator);
    elevator.setDefaultCommand(elevatorControl);
    autoIntake = new AutoIntake(intake, intakearms, camera);
    autoIntake.addRequirements(intake, intakearms, camera);
    autoShoot2Other = new AutoShoot2Other(shooter, intake, camera, driveTrain);
    autoShoot2Other.addRequirements(shooter, intake, camera, driveTrain);

    AutoChooser = new SendableChooser<String>();
    AutoChooser.setDefaultOption("GameDefault", "GameDefault");
    AutoChooser.addOption("GameOther", "GameOther");
    AutoChooser.addOption("Test Path", "Test Path");
    AutoChooser.addOption("Test Command", "Test Command");
    SmartDashboard.putData("Autonomous", AutoChooser);

    timerChooser = new SendableChooser<Boolean>();
    timerChooser.setDefaultOption("Timer Off", false);
    timerChooser.addOption("Timer On", true);
    SmartDashboard.putData("Timer", timerChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * tells the robot if you want the timer on or not
   * 
   * @return timer state
   */
  public static boolean getTimerState() {
    return timerChooser.getSelected();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton IntakeBallButton = new JoystickButton(manipulatorJoystick, Constants.IntakeBallButtonID);
    IntakeBallButton.whileHeld(new IntakeBall(intake));//intake ball

    JoystickButton SpitOutBallButton = new JoystickButton(manipulatorJoystick, Constants.SpitOutBallButtonID);
    SpitOutBallButton.whileHeld(new SpitOutBall(intake));//spit out ball

    JoystickButton ShootBall1Button = new JoystickButton(manipulatorJoystick, Constants.ShootBall1ButtonID);
    ShootBall1Button.whileHeld(new ShootBall1(shooter));//shoot ball speed1

    JoystickButton ShootBall2Button = new JoystickButton(manipulatorJoystick, Constants.ShootBall2ButtonID);
    ShootBall2Button.whileHeld(new ShootBall2(shooter));//shoot ball speed2

    JoystickButton ShootBall3Button = new JoystickButton(manipulatorJoystick, Constants.ShootBall3ButtonID);
    ShootBall3Button.whileHeld(new ShootBall3(shooter));//shoot ball speed3

    JoystickButton ShootBall4Button = new JoystickButton(manipulatorJoystick, Constants.ShootBall4ButtonID);
    ShootBall4Button.whileHeld(new ShootBallAuto(shooter));//shoot ball Auto

    JoystickButton FeederButton = new JoystickButton(manipulatorJoystick, Constants.FeederButtonID);
    FeederButton.whileHeld(new FeedBall(intake));//feed ball into shooter

    JoystickButton IntakeArmsUpButton = new JoystickButton(driverJoystick, Constants.IntakeArmsUpButtonID);
    IntakeArmsUpButton.whenPressed(new IntakeArmsUp(intakearms));//raise intake arms

    JoystickButton IntakeArmsDownButton = new JoystickButton(driverJoystick, Constants.IntakeArmsDownButtonID);
    IntakeArmsDownButton.whenPressed(new IntakeArmsDown(intakearms));//lower intake arms

    JoystickButton TrackTargetButton = new JoystickButton(manipulatorJoystick, Constants.TrackTargetButtonID);
    TrackTargetButton.whileHeld(new TrackTarget(camera));//Track Target

  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   String Selected = AutoChooser.getSelected();

    if(Selected == "Test Command"){
     return test;
    }else if(Selected == "GameDefault"){
      
     RamseteCommand GameDefaultcommand = new RamseteCommand(
      Robot.getGameDefaultTrajectory(), 
        driveTrain::getPose,
        new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
        driveTrain.getFeedForward(),
        driveTrain.getKinematics(),
        driveTrain::getSpeeds,
        driveTrain.getleftPidController(),
        driveTrain.getrightPidController(),
        driveTrain::tankDriveVolts,
        driveTrain
        );

      driveTrain.resetOdometry(Robot.getGameDefaultTrajectory().getInitialPose());

      return autoshoot1.
      andThen(GameDefaultcommand.raceWith(autoIntake)). 
      andThen(() -> driveTrain.tankDriveVolts(0, 0)).
      andThen(autoshoot2);
    
    }else if(Selected == "GameOther"){
      return autoShoot2Other;
    }else{
      
   RamseteCommand TestCommand = new RamseteCommand(
    Robot.getTestTrajectory(), 
    driveTrain::getPose,
    new RamseteController(Constants.kRamseteB,Constants.kRamseteZeta),
    driveTrain.getFeedForward(),
    driveTrain.getKinematics(),
    driveTrain::getSpeeds,
    driveTrain.getleftPidController(),
    driveTrain.getrightPidController(),
    driveTrain::tankDriveVolts,
    driveTrain
    );

    driveTrain.resetOdometry(Robot.getTestTrajectory().getInitialPose());

      return TestCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
    }
  }
}
