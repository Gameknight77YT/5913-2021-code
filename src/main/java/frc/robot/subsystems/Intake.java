// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final WPI_VictorSPX feederWheel = new WPI_VictorSPX(Constants.FeederWheelID);
  private final WPI_VictorSPX Brushes = new WPI_VictorSPX(Constants.BrushesID);
  private final WPI_VictorSPX intakeWheels = new WPI_VictorSPX(Constants.IntakeWheels);
  /** Creates a new Intake. */
  public Intake() {
    //nothing here lol
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void IntakeBall(double brushSpeed,double SuckSpeed) {
    Brushes.set(-brushSpeed);
    intakeWheels.set(-SuckSpeed);
  }

  public void SpitOutBall(double brushSpeed,double SuckSpeed) {
    Brushes.set(brushSpeed);
    intakeWheels.set(SuckSpeed);
  }

  public void FeedBall(double brushSpeed,double FeederSpeed) {
    Brushes.set(-brushSpeed);
    feederWheel.set(FeederSpeed);
  }

  public void IntakeAndShoot(double brushSpeed, double SuckSpeed, double FeederSpeed){
    Brushes.set(-brushSpeed);
    intakeWheels.set(-SuckSpeed);
    feederWheel.set(FeederSpeed);
  }

  public void Stop(){
    Brushes.stopMotor();
    intakeWheels.stopMotor();
    feederWheel.stopMotor();
  }


}
