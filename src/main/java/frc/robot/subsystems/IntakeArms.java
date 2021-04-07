// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArms extends SubsystemBase {
  private final DoubleSolenoid intakeArms = new DoubleSolenoid(Constants.IntakeArmsForwardID, Constants.IntakeArmsBackwardID);
  /** Creates a new IntakeArms. */
  public IntakeArms() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    resetIntakeArms();
  }

  public void lowerIntakeArms(){
    intakeArms.set(Value.kForward);
  }

  public void raiseIntakeArms(){
    intakeArms.set(Value.kReverse);
  }

  public void resetIntakeArms(){
    intakeArms.set(Value.kOff);
  }
  
}
