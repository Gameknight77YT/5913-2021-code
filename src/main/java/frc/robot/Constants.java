// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // IDs
  public static final int CameraID = 0;
  public static final int LeftMasterID = 1;
  public static final int RightMasterID = 2;
  public static final int LeftSlaveID = 4;
  public static final int RightSlaveID = 3;
  public static final int ElevatorMasterID = 9;
  public static final int ElevatorSlaveID = 10;
  public static final int FeederWheelID = 13;
  public static final int BrushesID = 17;
  public static final int IntakeWheels = 14;
  public static final int TopShooterID = 8;
  public static final int MainShooterID = 7;
  public static final int TurretControlID = 11;
  public static final int RatchetArmForwardID = 4;
  public static final int RatchetArmBackwardID = 1;
  public static final int IntakeArmsForwardID = 5;
  public static final int IntakeArmsBackwardID = 7;
  public static final int DriverJoystickID = 0;
  public static final int driverjoystickX = 0;
  public static final int driverjoystickY = 1;
  public static final int ManipulatorJoystickZAxisID = 2;
  public static final int ManipulatorJoystickID = 1;
  public static final int ShootBall1ButtonID = 7;
  public static final int ShootBall2ButtonID = 9;
  public static final int ShootBall3ButtonID = 11;
  public static final int ShootBall4ButtonID = 3;
  public static final int FeederButtonID = 2;
  public static final int IntakeBallButtonID = 5;
  public static final int SpitOutBallButtonID = 8;
  public static final int IntakeArmsUpButtonID = 2;
  public static final int IntakeArmsDownButtonID = 1;
  public static final int TrackTargetButtonID = 1;
  public static final int elevatorDownButtonID = 12;
  public static final int elevatorUpButtonID = 10;
  public static final int releaseElevatorButtonID = 6;
  public static final int ActivateTurnTurret = 4;

  // speeds
  public static final double speedY = .75;
  public static final double speedX = .65;
  public static final double AutonomousSpeed = 0.5;
  public static final double TurretSpeed = -.35;
  public static final double elevatorspeed = .4;
  public static final double SuckSpeed = 0.65;
  public static final double BrushsSpeed = 0.8;
  public static final double FeederSpeed = -0.7;
  public static final double ShooterSpeed1 = 20900;// 7
  public static final double ShooterSpeed2 = 22000;// 9
  public static final double ShooterSpeed3 = 23500;// 11
  public static final double ShooterSpeed4 = 27500;// 3

  // encoder counts/ times
  public static final double AutoTrackTime = 1.5;// keep this equal to the ShooterSpinupTime
  public static final double ShooterSpinupTime = 1.5;// keep this equal to the AutoTrackTime
  public static final double AutoShootTime = 4;// must be more the ShooterSpinupTime and AutoTrackTime
  public static final int DriveDistance = 70;

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or
  // theoretically for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining
  // these
  // values for your robot.
  public static final double ks = 0.706;// Volts
  public static final double kv = 2.1;// VoltSecondsPerMeter
  public static final double ka = 0.372; // VoltSecondsSquaredPerMeter

  public static final double WheelBaseWith = Units.inchesToMeters(25);

  public static final double kMaxSpeedMetersPerSecond = 4;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  // Reasonable baseline values for a RAMSETE follower in units of meters and
  // seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  // PID controller values
  public static final double kp = 2.81;
  public static final double ki = 0;
  public static final double kd = 0;

  // other constants
  public static final double WheelDiameter = 6;
  public static final double EncoderConstant = 668.897;

}
