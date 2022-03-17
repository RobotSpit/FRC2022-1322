// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.calibrations.K_SHOT;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CA_ShootLo extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private Camera cameraSubsystem;
  private Timer timeSinceBallsLeftAdvPstn;
  private double targetDistance;
  private double dsrdShooterSpeed;
  
  /** Creates a new CA_ShootLo - Low Goal. */
  public CA_ShootLo(ShooterSubsystem shooterSubsystem,  IntakeSubsystem intakeSubsystem, Camera cameraSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.cameraSubsystem = cameraSubsystem;
    timeSinceBallsLeftAdvPstn = new Timer();
    addRequirements(intakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Robot, Shoot! Low Goal.");
    intakeSubsystem.getAdvanceMtr().setNeutralMode(NeutralMode.Coast);
    timeSinceBallsLeftAdvPstn.reset();
    timeSinceBallsLeftAdvPstn.stop();

    // Determine Desired Target Speed as a function of target distance
    targetDistance = cameraSubsystem.getDistanceToCenterOfHoop();
    dsrdShooterSpeed = shooterSubsystem.dtrmnShooterSpd(targetDistance, false);

    // Command Shooter to Desired Shooter Target Speed
    shooterSubsystem.runShooterAtSpd(dsrdShooterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // When shooter is at speed, feed the balls for shot, otherwise wait.
    shooterSubsystem.dtrmnShooterAtSpd(dsrdShooterSpeed);

    if (shooterSubsystem.isShooterAtSpd()) {
      System.out.println("Command Intakes!");
      intakeSubsystem.runAdvanceAtPwr(0.9);
      intakeSubsystem.runIntakeAtPwr(0.9);
    } else {
      intakeSubsystem.stopAdvanceMtr();
      intakeSubsystem.stopIntakeMtr();
    }
    
    if ((intakeSubsystem.getBallAdvPstn1() == false) && (intakeSubsystem.getBallAdvPstn2() == false)) {
      timeSinceBallsLeftAdvPstn.start();      
    } else {
      timeSinceBallsLeftAdvPstn.reset();
      timeSinceBallsLeftAdvPstn.stop();        
    }
    if (timeSinceBallsLeftAdvPstn.get() >= K_SHOT.KeSHOT_t_IntakeArmRaiseDlyShoot) {
      intakeSubsystem.raiseIntakeArms();
    }

    if (K_SHOT.KeSHOT_b_DebugEnbl == true) {
      SmartDashboard.putNumber("Shooter Act Spd: ",    (shooterSubsystem.getSpd()));
      SmartDashboard.putNumber("Shooter Tgt Spd: ",    (dsrdShooterSpeed));
      SmartDashboard.putBoolean("Shooter At Spd: ",    (shooterSubsystem.isShooterAtSpd()));
      SmartDashboard.putNumber("Shooter Tgt Dist: ",   (targetDistance));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print("Shooter dead");
     intakeSubsystem.stopAdvanceMtr();
     intakeSubsystem.stopIntakeMtr();
     shooterSubsystem.stopShooterMtr();
     timeSinceBallsLeftAdvPstn.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timeSinceBallsLeftAdvPstn.get() >= K_SHOT.KeSHOT_t_PostLaunchRunTime);
  }
}
