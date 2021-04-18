// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonSRX mainShooter = new WPI_TalonSRX(Constants.MainShooterID);
  private final WPI_TalonSRX topShooter = new WPI_TalonSRX(Constants.TopShooterID);
  /** Creates a new Shooter. */
  public Shooter() {
    
    mainShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    topShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    mainShooter.clearStickyFaults(10);
    topShooter.clearStickyFaults(10);

    //config PIDs
    mainShooter.config_kF(0, 0.0265, 10);
    mainShooter.config_kP(0, 0.65, 10);
    mainShooter.config_kI(0, 0.0002, 10);
    mainShooter.config_kD(0, 6, 10);
    mainShooter.config_IntegralZone(0, 100, 10);

    topShooter.config_kF(0, 0.02764865, 10);
    topShooter.config_kP(0, 0.05, 10);
    topShooter.config_kI(0, 0, 10);
    topShooter.config_kD(0, 0, 10);

    //reset encoders
    mainShooter.setSelectedSensorPosition(0, 0, 10);
    topShooter.setSelectedSensorPosition(0, 0, 10);

    //set configs
    mainShooter.setNeutralMode(NeutralMode.Coast);
    topShooter.setNeutralMode(NeutralMode.Coast);

  }

  @Override
  public void periodic() { 
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("MainShooter Speed", mainShooter.getSelectedSensorVelocity());
    //SmartDashboard.putNumber("TopShooter Speed", topShooter.getSelectedSensorVelocity());
  }

  /** Shoots Ball with Speed Preset 1  */
  public void ShootBallSpeed1(Double speed) {
    mainShooter.set(ControlMode.Velocity, speed);
    topShooter.set(ControlMode.Velocity, (speed*-1) - 3000);
  }

  /** Shoots Ball with Speed Preset 2  */
  public void ShootBallSpeed2(double speed) {
    mainShooter.set(ControlMode.Velocity, speed);
    topShooter.set(ControlMode.Velocity, (speed *-1)+ 1000);
  }

  /** Shoots Ball with Speed Preset 3  */
  public void ShootBallSpeed3(double speed) {
    mainShooter.set(ControlMode.Velocity, speed);
    topShooter.set(ControlMode.Velocity, speed *-1);
  }

  /** Shoots Ball with Speed Preset 4  */
  public void ShootBallSpeed4(double speed) {
    mainShooter.set(ControlMode.Velocity, speed);
    topShooter.set(ControlMode.Velocity, (speed *-1)+500);
  }

  /** Stops Shooter */
  public void StopShooter(){
    mainShooter.stopMotor();
    topShooter.stopMotor();
  }













}
