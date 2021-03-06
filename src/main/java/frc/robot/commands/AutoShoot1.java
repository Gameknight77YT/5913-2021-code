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

public class AutoShoot1 extends CommandBase {
  Shooter shooter;
  Intake intake;
  Camera camera;
  Timer timer = new Timer();
  private boolean finish = false;
  /** Creates a new AutoShoot. */
  public AutoShoot1(Shooter s , Intake i, Camera c) {
    shooter = s;
    intake = i;
    camera = c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter,intake,camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    camera.Reset();
    while(timer.get() < Constants.ShooterSpinupTime){
      shooter.ShootBallSpeed1(Constants.ShooterSpeed1);
      camera.AutoTrack();
    }
    while(timer.get() > Constants.ShooterSpinupTime & timer.get() < Constants.AutoShootTime){
      shooter.ShootBallSpeed1(Constants.ShooterSpeed1);
      intake.FeedBall(Constants.BrushsSpeed, Constants.FeederSpeed);
      camera.AutoTrack();
    }
    finish = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.StopShooter();
    intake.Stop();
    camera.Reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
