// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArms;

public class AutoIntake extends CommandBase {
  IntakeArms intakeArms;
  Intake intake;
  /** Creates a new AutoIntake. */
  public AutoIntake(Intake i,IntakeArms ia) {
    intake = i;
    intakeArms = ia;
    addRequirements(intake, intakeArms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeArms.lowerIntakeArms();
    intake.IntakeAndShoot(Constants.BrushsSpeed,Constants.SuckSpeed, Constants.FeederSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.Stop();
    intakeArms.resetIntakeArms();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
