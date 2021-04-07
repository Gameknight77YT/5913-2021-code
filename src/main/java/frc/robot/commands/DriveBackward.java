// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveBackward extends CommandBase {
  private boolean finish = false;
  DriveTrain driveTrain;
  /** Creates a new DriveBackward. */
  public DriveBackward(DriveTrain dt) {
    driveTrain = dt;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.ClearDriveEncoders();
    driveTrain.driveBackward(Constants.AutonomousSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(driveTrain.GetLeftMasterEncoderPose() < Constants.driveBackwardDistance){
     driveTrain.driveBackward(Constants.AutonomousSpeed);
   }else if(driveTrain.GetLeftMasterEncoderPose() > Constants.driveBackwardDistance){
     finish = true;
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopmotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}