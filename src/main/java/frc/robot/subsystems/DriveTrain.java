// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {
  WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.LeftMasterID);
  WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.RightMasterID);
  WPI_TalonFX leftSlave = new WPI_TalonFX(Constants.LeftSlaveID);
  WPI_TalonFX rightSlave = new WPI_TalonFX(Constants.RightSlaveID);

  SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMaster, leftSlave);
  SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMaster, rightSlave);

  //DifferentialDrive robotDrive = new DifferentialDrive(leftMaster, rightMaster);

  DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.WheelBaseWith);
  DifferentialDriveOdometry Odometry = new DifferentialDriveOdometry(Robot.getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka);

  PIDController leftPidController = new PIDController(Constants.kp, Constants.ki, Constants.kd);
  PIDController rightPidController = new PIDController(Constants.kp, Constants.ki, Constants.kd);

  Pose2d pose = new Pose2d();
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {

  //slave setups
  //leftSlave.follow(leftMaster);
  //rightSlave.follow(rightMaster);

  //invert setup
  leftMaster.setInverted(false);
  leftSlave.setInverted(false);
  rightSlave.setInverted(true);
  rightMaster.setInverted(true);

  //init encoders
  leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

  leftMaster.clearStickyFaults(10);
  rightMaster.clearStickyFaults(10);

  leftMaster.configOpenloopRamp(1);
  rightMaster.configOpenloopRamp(1);
  leftSlave.configOpenloopRamp(1);
  rightSlave.configOpenloopRamp(1);

  leftMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 175, 5));
  rightMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 175, 5));
  leftSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 175, 5));
  rightSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 175, 5));

  //leftMaster.setSafetyEnabled(false);
  //rightMaster.setSafetyEnabled(false);
  //leftSlave.setSafetyEnabled(false);
  //rightSlave.setSafetyEnabled(false);

  }

  @Override
  public void periodic() {
    GetRightMasterEncoderSpeed();
    GetLeftMasterEncoderSpeed();
    GetRightMasterEncoderPose();
    GetLeftMasterEncoderPose();
    //SmartDashboard.putNumber("LeftEncoderPose", GetLeftMasterEncoderPose()/Constants.EncoderConstant);
    //SmartDashboard.putNumber("RightEncoderPose", GetRightMasterEncoderPose()/Constants.EncoderConstant);
    //SmartDashboard.putNumber("LeftEncoderSpeed", GetLeftMasterEncoderSpeed());
    //SmartDashboard.putNumber("RightEncoderSpeed", GetRightMasterEncoderSpeed());

    pose = Odometry.update(
    Robot.getHeading(),
    GetLeftMasterEncoderPose(),
    GetRightMasterEncoderPose()
    );
  }
  public void resetOdometry(Pose2d pose2D) {
    ClearDriveEncoders();
    Odometry.resetPosition(pose2D, Robot.getRotation2d());
  }
  

  

  public DifferentialDriveWheelSpeeds getSpeeds() {
   return new DifferentialDriveWheelSpeeds(
    leftMaster.getSelectedSensorVelocity() / Constants.GearRatio * 2 * Math.PI * Units.inchesToMeters(Constants.WheelRadiusInches) / 60,
    rightMaster.getSelectedSensorVelocity() / Constants.GearRatio * 2 * Math.PI * Units.inchesToMeters(Constants.WheelRadiusInches) / 60
   );
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  public PIDController getleftPidController() {
    return leftPidController;
  }
  
  public PIDController getrightPidController() {
    return rightPidController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return Kinematics;
  }

  public Pose2d getPose() {
     return Odometry.getPoseMeters();
  }

  public void reset() {
    Odometry.resetPosition(new Pose2d(), Robot.getHeading());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
  }

  

  /** Makes Robot Go Brrrrrrr */
  public void DriveWithJoystick(Joystick driverJoystick) {
    //robotDrive.arcadeDrive(driverJoystick.getRawAxis(Constants.driverjoystickX)*Constants.speedX,driverJoystick.getRawAxis(Constants.driverjoystickY)*Constants.speedY);
    double joy_y = driverJoystick.getRawAxis(Constants.driverjoystickX)*Constants.speedX;
    double joy_x = -driverJoystick.getRawAxis(Constants.driverjoystickY)*Constants.speedY;
    double threshold = .2;
    double leftMotorOutput;
    double rightMotorOutput;

    double xSpeed = MathUtil.clamp(joy_x, -1.0, 1.0);
    xSpeed = applyDeadband(joy_x, threshold);
    double zRotation = MathUtil.clamp(joy_y, -1.0, 1.0);
    zRotation = applyDeadband(joy_y, threshold);
    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }
    leftMotors.set(MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * 1);
    rightMotors.set(MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * 1);
  }
  
  /** Applys a Deadband */
  public double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public void Drive(double leftMotorOutput, double rightMotorOutput){
    leftMotors.set(leftMotorOutput);
    rightMotors.set(rightMotorOutput);
  }
  

  public void stopmotors(){
    rightMotors.stopMotor();
    leftMotors.stopMotor();
  }

  public double GetLeftMasterEncoderPose() {
    return nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition());
  }

  public double GetRightMasterEncoderPose() {
    return nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition());
  }

  public double GetLeftMasterEncoderSpeed() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public double GetRightMasterEncoderSpeed() {
    return rightMaster.getSelectedSensorVelocity();
  }

  public void ClearDriveEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  /**sets neutral mode
   * 
   * @param mode 1 = coast, 0 = brake
   */
  public void SetMotorMode(double mode){
    if(mode==1){

    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);

    }else if(mode==0){

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
    }
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / 2048;
    double wheelRotations = motorRotations / Constants.GearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.WheelRadiusInches));
    return positionMeters;
  }
  
}
