// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeAndShoot extends CommandBase {
  Intake intake;
  Shooter shooter;
  Camera camera;
  private boolean finish = false;
  Timer timer = new Timer();
  /** Creates a new IntakeAndShoot. */
  public IntakeAndShoot(Camera c, Shooter s,Intake i) {
    shooter = s; 
    intake = i;
    camera = c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get()<Constants.ShooterSpinupTime){
      shooter.ShootBallSpeed1(Constants.ShooterSpeed1);
    }else{
    shooter.ShootBallSpeed1(Constants.ShooterSpeed1);
    camera.Track();
    intake.IntakeAndShoot(Constants.BrushsSpeed,Constants.SuckSpeed, Constants.FeederSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.StopShooter();
    intake.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
