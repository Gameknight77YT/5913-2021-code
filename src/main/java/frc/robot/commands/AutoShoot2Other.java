// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShoot2Other extends CommandBase {
  Shooter shooter;
  Intake intake;
  Camera camera;
  DriveTrain driveTrain;
  Timer timer = new Timer(); 
  private boolean finish = false;
  /** Creates a new AutoShoot. */
  public AutoShoot2Other(Shooter s , Intake i, Camera c, DriveTrain dt) {
    shooter = s;
    intake = i;
    camera = c;
    driveTrain = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter,intake,camera,driveTrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.ClearDriveEncoders();
    timer.reset();
    timer.start();
    while(timer.get() < Constants.ShooterSpinupTime){
      shooter.ShootBallSpeed1(Constants.ShooterSpeed1);
      camera.AutoTrack();
      
    }
    while(timer.get() > Constants.ShooterSpinupTime & timer.get() < Constants.AutoShootTime){
      shooter.ShootBallSpeed1(Constants.ShooterSpeed1);
      intake.FeedBall(Constants.BrushsSpeed, Constants.FeederSpeed);
      camera.AutoTrack();
      driveTrain.stopmotors();
      driveTrain.SetMotorMode(0);
    }
    while(timer.get() > Constants.AutoShootTime && timer.get() < 14){
    shooter.StopShooter();
    intake.Stop();
      while(driveTrain.GetLeftMasterEncoderPose() < 70 ){
        driveTrain.Drive(.2, .2);
      }
      while(driveTrain.GetLeftMasterEncoderPose() > 70){
        driveTrain.stopmotors();
        driveTrain.SetMotorMode(0);
        finish = true;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    camera.Reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
