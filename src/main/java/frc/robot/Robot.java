// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer container;
  private Timer timer;
  private Timer timer2;
  private DriveTrain driveTrain;
  static Trajectory TestTrajectory = new Trajectory();
  static Trajectory Game1 = new Trajectory();
  static Trajectory Game2 = new Trajectory();
  static Trajectory GameDefault = new Trajectory();
  static Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS1);

  /**
   * This function is run when the robot is first ed up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    calibrate();
    resetGyro();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    container = new RobotContainer();
    timer = new Timer();
    timer2 = new Timer();
    driveTrain = new DriveTrain();
    SmartDashboard.putData(CommandScheduler.getInstance());
    // init Trajectorys
    InitTrajectorys();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void calibrate() {
    gyro.calibrate();
  }

  public static Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }
  public static Rotation2d getRotation2d(){
    return gyro.getRotation2d();
  }

  public void InitTrajectorys() {
    try {
      Path TesttrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Test.wpilib.json");
      TestTrajectory = TrajectoryUtil.fromPathweaverJson(TesttrajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + "paths/Test.wpilib.json", ex.getStackTrace());
    }
       String Game1JSON = "paths/Game1.wpilib.json";
     try {
      Path Game1Path = Filesystem.getDeployDirectory().toPath().resolve(Game1JSON);
      Game1 = TrajectoryUtil.fromPathweaverJson(Game1Path);
     } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Game1JSON, ex.getStackTrace());
     }

       String Game2JSON = "paths/Game2.wpilib.json";
     try {
      Path Game2Path = Filesystem.getDeployDirectory().toPath().resolve(Game2JSON);
      Game2 = TrajectoryUtil.fromPathweaverJson(Game2Path);
     } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + Game2JSON, ex.getStackTrace());
     }

       String GameDefaultJSON = "paths/GameDefault.wpilib.json";
     try {
      Path GameDefaultPath = Filesystem.getDeployDirectory().toPath().resolve(GameDefaultJSON);
      GameDefault = TrajectoryUtil.fromPathweaverJson(GameDefaultPath);
     } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + GameDefaultJSON, ex.getStackTrace());
     }
  }

  public static Trajectory getTestTrajectory() {
    return TestTrajectory;
  }
  public static Trajectory getGame1Trajectory() {
    return Game1;
  }
  public static Trajectory getGame2Trajectory() {
    return Game2;
  }
  public static Trajectory getGameDefaultTrajectory() {
    return GameDefault;
  }
  
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("gyroValue", gyro.getAngle());
    SmartDashboard.putNumber("gyroRate", gyro.getRate());
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    timer.stop();
    SmartDashboard.putNumber("Time", timer.get());
    driveTrain.SetMotorMode(0);
    timer2.start();
    if(timer2.get()>4){
      driveTrain.SetMotorMode(1);
      timer2.stop();
      timer2.reset();
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    driveTrain.SetMotorMode(1);
    container.getAutonomousCommand().schedule();    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    timer.reset();
    timer.start();
    driveTrain.SetMotorMode(1);
    // This makes sure that the autonomous stops running when
    // teleop s running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //driveTrain.DriveWithJoystick(RobotContainer.driverJoystick);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the  of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
