// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.calibrations.K_SHOT;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualShootHi extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;

  
  /** Creates a new ManualShootHi - High Goal. */
  public ManualShootHi(ShooterSubsystem shooterSubsystem,  IntakeSubsystem intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Robot, Shoot! High Goal.");
    shooterSubsystem.runShooterAtSpd(K_SHOT.KeSHOT_n_TgtLaunchCmdHiGoal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // When shooter is at speed, feed the balls for shot, otherwise wait.
    shooterSubsystem.dtrmnShooterAtSpd(K_SHOT.KeSHOT_n_TgtLaunchCmdHiGoal);

    if (shooterSubsystem.isShooterAtSpd()) {
      intakeSubsystem.runAdvanceAtPwr(0.9);
      intakeSubsystem.runIntakeAtPwr(0.9);
    } else {
      intakeSubsystem.stopAdvanceMtr();
      intakeSubsystem.stopIntakeMtr();
    }
    
    if (K_SHOT.KeSHOT_b_DebugEnbl == true) {
      SmartDashboard.putNumber("Shooter Speed: ",     (shooterSubsystem.getSpd()));
      SmartDashboard.putNumber("Shooter Target: ",    (K_SHOT.KeSHOT_n_TgtLaunchCmdHiGoal));
      SmartDashboard.putBoolean("Shooter At Speed: ", (shooterSubsystem.isShooterAtSpd()));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print("Shooter dead");
     intakeSubsystem.stopAdvanceMtr();
     intakeSubsystem.stopIntakeMtr();
     shooterSubsystem.stopShooterMtr();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
