// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

  private final WPI_TalonSRX elevatorMaster = new WPI_TalonSRX(Constants.ElevatorMasterID);
  private final WPI_VictorSPX elevatorSlave = new WPI_VictorSPX(Constants.ElevatorSlaveID);
  private final DoubleSolenoid ratchetArm = new DoubleSolenoid(Constants.RatchetArmForwardID, Constants.RatchetArmBackwardID);


  /** Creates a new Elevator. */
  public Elevator() {

    
    elevatorSlave.follow(elevatorMaster);

    elevatorSlave.setInverted(InvertType.FollowMaster);
    //init encoder
    elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    
    elevatorMaster.setSelectedSensorPosition(0);
    elevatorMaster.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElevatorEncoder", getElevatorEncoderPosition());
  }

  public void elevatorup(double speed) {
    if(RobotContainer.driverJoystick.getRawButton(Constants.releaseElevatorButtonID)==true){
    elevatorMaster.set(ControlMode.PercentOutput, speed);
    }
  }
  
  public void ReleaseElevator(){
    ratchetArm.set(Value.kReverse);
  }

  public void elevatordown(double speed) {
    elevatorMaster.set(ControlMode.PercentOutput, -speed);
    ratchetArm.set(Value.kForward);
  }

  public void elevatorstop() {
    elevatorMaster.set(0);
  }

  public void RatchetOff(){
    ratchetArm.set(Value.kOff);
  }

  public  double getElevatorEncoderPosition() {
	return elevatorMaster.getSelectedSensorPosition();
  }
}
