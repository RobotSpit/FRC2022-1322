// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.calibrations.K_INTK;
import frc.robot.calibrations.K_SHOT;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualShootLo extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private  Timer shooterShutOffTimer = new Timer();

  
  /** Creates a new ManualShootLo - Low Goal. */
  public ManualShootLo(ShooterSubsystem shooterSubsystem,  IntakeSubsystem intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem, shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Robot, Shoot! Low Goal.");
    shooterSubsystem.runShooterAtSpd(K_SHOT.KeSHOT_n_TgtLaunchCmdLoGoal);
    shooterShutOffTimer.reset();
    shooterShutOffTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // When shooter is at speed, feed the balls for shot, otherwise wait.
    shooterSubsystem.dtrmnShooterAtSpd(K_SHOT.KeSHOT_n_TgtLaunchCmdLoGoal);

    if (shooterSubsystem.isShooterAtSpd()) {
      System.out.println("Command Intakes!");
//      intakeSubsystem.runAdvanceAtSpd(K_INTK.KeINTK_n_TgtAdvanceCmdShoot);
//      intakeSubsystem.runIntakeAtSpd(K_INTK.KeINTK_n_TgtIntakeCmdShoot);
      intakeSubsystem.runAdvanceAtPwr(0.3);
      intakeSubsystem.runIntakeAtPwr(0.3);
      shooterShutOffTimer.start();
    } else {
      intakeSubsystem.stopAdvanceMtr();
      intakeSubsystem.stopIntakeMtr();
    }
    
    if ((intakeSubsystem.getBallAdvPstn1() == true) ||(intakeSubsystem.getBallAdvPstn2() == true)) {
      shooterShutOffTimer.reset();
    }

    SmartDashboard.putNumber("Shooter Speed: ",     (shooterSubsystem.getSpd()));
    SmartDashboard.putNumber("Shooter Target: ",    (K_SHOT.KeSHOT_n_TgtLaunchCmdLoGoal));
    SmartDashboard.putBoolean("Shooter At Speed: ", (shooterSubsystem.isShooterAtSpd()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print("Shooter dead");
     intakeSubsystem.stopAdvanceMtr();
     intakeSubsystem.stopIntakeMtr();
     shooterSubsystem.stopShooterMtr();
     shooterShutOffTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // boolean condMet = (shooterShutOffTimer.get() >= K_SHOT.KeSHOT_t_PostLaunchRunTime);
    return false;
  }
}
