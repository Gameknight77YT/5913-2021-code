// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final WPI_TalonSRX elevatorMaster = new WPI_TalonSRX(Constants.ElevatorMasterID);
  private final WPI_VictorSPX elevatorSlave = new WPI_VictorSPX(Constants.ElevatorSlaveID);
  private final DoubleSolenoid ratchetArm = new DoubleSolenoid(Constants.RatchetArmForwardID, Constants.RatchetArmBackwardID);


  /** Creates a new Elevator. */
  public Elevator() {
    elevatorSlave.follow(elevatorMaster);

    //init encoder
    elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    elevatorMaster.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("ElevatorEncoder", getElevatorEncoderPosition());
  }

  public void ControlElevator(Joystick manipulatorJoystick){
    double elevatorSpeed = 0; 
      if (manipulatorJoystick.getRawButton(Constants.releaseElevatorButtonID)==true //up
      && manipulatorJoystick.getRawButton(Constants.elevatorUpButtonID)){
        elevatorSpeed = 1;
      }
      if (manipulatorJoystick.getRawButton(Constants.elevatorDownButtonID)==true){  //down
        elevatorSpeed = -1;
      }
      elevatorMaster.set(ControlMode.PercentOutput, elevatorSpeed);
    ratchetArm.set(Value.kOff);
    if (manipulatorJoystick.getRawButton(Constants.releaseElevatorButtonID)==true){ //release 
        ratchetArm.set(Value.kReverse);
    }    
    if (manipulatorJoystick.getRawButton(Constants.elevatorDownButtonID)==true){ //down
        ratchetArm.set(Value.kForward);
    }
  }

  public  double getElevatorEncoderPosition() {
	return elevatorMaster.getSelectedSensorPosition();
  }
}
