// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.calibrations.K_BALL;
import frc.robot.calibrations.K_SHOT;
import frc.robot.subsystems.BallSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualShoot extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private BallSubsystem ballSubsystem;
  private XboxController auxStick;
  private  Timer shooterShutOffTimer = new Timer();

  
  /** Creates a new ManualShoot. */
  public ManualShoot(ShooterSubsystem shooterSubsystem,  BallSubsystem ballSubsystem, XboxController auxStick) {
  //  addRequirements(ballSubsystem, shooterSubsystem, auxStick);
    this.shooterSubsystem = shooterSubsystem;
    this.ballSubsystem = ballSubsystem;
    this.auxStick = auxStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.pidShooterSpd(true);
    shooterShutOffTimer.reset();
    shooterShutOffTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // When shooter is at speed, feed the balls for shot, otherwise wait.
    if (shooterSubsystem.isShooterAtSpd()) {
      ballSubsystem.runAdvanceAtSpd(K_BALL.KeBALL_n_TgtAdvanceCmd);
      ballSubsystem.runIntakeAtSpd(K_BALL.KeBALL_n_TgtIntakeCmd);
      shooterShutOffTimer.start();
    } else {
      ballSubsystem.runAdvanceAtSpd(0);
      ballSubsystem.runIntakeAtSpd(0);
    }
    
    if ((auxStick.getAButton() == true) || (ballSubsystem.detectBallAdvIntake() == true) ||
        (ballSubsystem.detectBallAdvOuttake() ==true)) {
      shooterShutOffTimer.reset();
    }

    SmartDashboard.putNumber("Shooter Speed: ", shooterSubsystem.getSpd());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballSubsystem.runAdvanceAtSpd(0);
    ballSubsystem.runIntakeAtSpd(0);
    shooterSubsystem.pidShooterSpd(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean condMet = (auxStick.getAButtonReleased() || (shooterShutOffTimer.get() >= K_SHOT.KeSHOT_t_PostLaunchRunTime));
    return condMet;
  }
}
