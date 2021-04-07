// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera;

public class TrackTarget extends CommandBase {
  Camera camera;
  Timer timer;
  /** Creates a new TrackTarget. */
  public TrackTarget(Camera c) {
    camera = c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.Track();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   camera.Reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
