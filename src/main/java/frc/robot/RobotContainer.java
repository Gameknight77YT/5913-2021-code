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
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ControlWithJoystick;
import frc.robot.commands.DriveBackward;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.FeedBall;
import frc.robot.commands.IntakeAndShoot;
import frc.robot.commands.IntakeArmsDown;
import frc.robot.commands.IntakeArmsUp;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ReleaseElevator;
import frc.robot.commands.ShootBall1;
import frc.robot.commands.ShootBall2;
import frc.robot.commands.ShootBall3;
import frc.robot.commands.ShootBall4;
import frc.robot.commands.SpitOutBall;
import frc.robot.commands.Test;
import frc.robot.commands.TrackAndShoot;
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
  private final ElevatorDown elevatordown;
  private final ElevatorUp elevatorup;
  private final ReleaseElevator releaseElevator;
  private final IntakeBall intakeball;
  private final SpitOutBall spitoutball;
  private final FeedBall feedball;
  private final ShootBall1 shootball1;
  private final ShootBall2 shootball2;
  private final ShootBall3 shootball3;
  private final ShootBall4 shootball4;
  private final AutoShoot autoshoot;
  private final IntakeArmsUp intakeArmsUp;
  private final IntakeArmsDown intakeArmsDown;
  private final TrackTarget trackTarget;
  private final TrackAndShoot trackAndShoot;
  private final IntakeAndShoot intakeAndShoot;
  private final Test test;
  private final DriveForward driveForward;
  private final DriveBackward driveBackward;

  //objects
  public static Joystick driverJoystick;
  public static Joystick manipulatorJoystick;
  SendableChooser<String> chooserString;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    //init subsystems
    driveTrain = new DriveTrain();
    elevator = new Elevator();
    intake = new Intake();
    shooter = new Shooter();
    intakearms = new IntakeArms();
    camera = new Camera();
    SmartDashboard.putData(driveTrain);
    SmartDashboard.putData(elevator);
    SmartDashboard.putData(intake);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(intakearms);
    SmartDashboard.putData(camera);

    //init objects
    driverJoystick = new Joystick(Constants.DriverJoystickID);
    manipulatorJoystick = new Joystick(Constants.ManipulatorJoystickID);

    //init commands
    driveWithJoystick = new DriveWithJoysticks(driveTrain);
    driveWithJoystick.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoystick);
    controlWithJoystick = new ControlWithJoystick(camera);
    controlWithJoystick.addRequirements(camera);
    camera.setDefaultCommand(controlWithJoystick);
    elevatorup = new ElevatorUp(elevator);
    elevatorup.addRequirements(elevator);
    elevatordown = new ElevatorDown(elevator);
    elevatordown.addRequirements(elevator);
    releaseElevator = new ReleaseElevator(elevator);
    releaseElevator.addRequirements(elevator);
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
    shootball4 = new ShootBall4(shooter);
    shootball4.addRequirements(shooter);
    feedball = new FeedBall(intake);
    feedball.addRequirements(intake);
    autoshoot = new AutoShoot(shooter, intake);
    autoshoot.addRequirements(shooter,intake);
    intakeArmsDown = new IntakeArmsDown(intakearms);
    intakeArmsDown.addRequirements(intakearms);
    intakeArmsUp = new IntakeArmsUp(intakearms);
    intakeArmsUp.addRequirements(intakearms);
    trackTarget = new TrackTarget(camera);
    trackTarget.addRequirements(camera);
    trackAndShoot = new TrackAndShoot(camera, shooter, intake);
    trackAndShoot.addRequirements(camera, shooter, intake);
    intakeAndShoot = new IntakeAndShoot(camera, shooter, intake);
    intakeAndShoot.addRequirements(shooter, intake, camera);
    test = new Test(driveTrain);
    test.addRequirements(driveTrain);
    driveForward = new DriveForward(driveTrain);
    driveForward.addRequirements(driveTrain);
    driveBackward = new DriveBackward(driveTrain);
    driveBackward.addRequirements(driveTrain);

    chooserString = new SendableChooser<String>();
    chooserString.setDefaultOption("GameDefault", "GameDefault");
    chooserString.addOption("Game1", "Game1");
    chooserString.addOption("Game2", "Game2");
    chooserString.addOption("Test", "paths/Test.wpilib.json");
    SmartDashboard.putData("Autonomous",chooserString);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton ElevatorUpButton = new JoystickButton(driverJoystick, (Constants.elevatorUpButtonID));
    ElevatorUpButton.whileHeld(new ElevatorUp(elevator));//brings elevator up

    JoystickButton releaseElevatorButton = new JoystickButton(driverJoystick, (Constants.releaseElevatorButtonID));
    releaseElevatorButton.whileHeld(new ReleaseElevator(elevator));
 
    JoystickButton ElevatorDownButton = new JoystickButton(driverJoystick, (Constants.elevatorDownButtonID));
    ElevatorDownButton.whileHeld(new ElevatorDown(elevator));//brings elevator down

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
    ShootBall4Button.whileHeld(new ShootBall4(shooter));//shoot ball speed4

    JoystickButton FeederButton = new JoystickButton(manipulatorJoystick, Constants.FeederButtonID);
    FeederButton.whileHeld(new FeedBall(intake));//feed ball into shooter

    JoystickButton IntakeArmsUpButton = new JoystickButton(driverJoystick, Constants.IntakeArmsUpButtonID);
    IntakeArmsUpButton.whenPressed(new IntakeArmsUp(intakearms));//raise intake arms

    JoystickButton IntakeArmsDownButton = new JoystickButton(driverJoystick, Constants.IntakeArmsDownButtonID);
    IntakeArmsDownButton.whenPressed(new IntakeArmsDown(intakearms));//lower intake arms

    JoystickButton TrackTargetButton = new JoystickButton(manipulatorJoystick, Constants.TrackTargetButtonID);
    TrackTargetButton.whileHeld(new TrackTarget(camera));//Track Target

    JoystickButton intakeAndShootButton = new JoystickButton(manipulatorJoystick, Constants.intakeAndShootButtonID);
    intakeAndShootButton.whileHeld(new IntakeAndShoot(camera, shooter, intake));//Intake And Shoot
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   String Selected = chooserString.getSelected();
     //return test;
    if(Selected=="Game1"){
      
     RamseteCommand Game1command = new RamseteCommand(
        Robot.getGame1Trajectory(), 
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

      driveTrain.resetOdometry(Robot.getGame1Trajectory().getInitialPose());

      return Game1command.raceWith(intakeAndShoot). 
      andThen(() -> driveTrain.tankDriveVolts(0, 0));

    }else if(Selected=="Game2"){
      
     RamseteCommand Game2command = new RamseteCommand(
        Robot.getGame2Trajectory(), 
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

      driveTrain.resetOdometry(Robot.getGame2Trajectory().getInitialPose());

      return Game2command.raceWith(intakeAndShoot). 
      andThen(() -> driveTrain.tankDriveVolts(0, 0));
      
    }else if(Selected=="GameDefault"){
      
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

      return GameDefaultcommand.//raceWith(intakeAndShoot). 
      andThen(() -> driveTrain.tankDriveVolts(0, 0));
    
    }else{
      
   RamseteCommand Defaultcommand = new RamseteCommand(
    Robot.getDefaultTrajectory(), 
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

    driveTrain.resetOdometry(Robot.getDefaultTrajectory().getInitialPose());

      return Defaultcommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
    }
  }
}
