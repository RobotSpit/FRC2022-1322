// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.calibrations.K_INTK;
import frc.robot.calibrations.K_SHOT;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ManualShoot extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private JoystickButton auxButton_A;
  private  Timer shooterClearTimer = new Timer();
  private  Timer shooterShutOffTimer = new Timer();

  
  /** Creates a new ManualShoot. */
  public ManualShoot(ShooterSubsystem shooterSubsystem,  IntakeSubsystem intakeSubsystem, JoystickButton auxButton_A) {
    //  addRequirements(intakeSubsystem, shooterSubsystem, auxStick);
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.auxButton_A = auxButton_A;
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
      intakeSubsystem.runAdvanceAtSpd(K_INTK.KeINTK_n_TgtAdvanceCmdShoot);
      intakeSubsystem.runIntakeAtSpd(K_INTK.KeINTK_n_TgtIntakeCmdShoot);
      shooterShutOffTimer.start();
    } else {
      intakeSubsystem.runAdvanceAtSpd(0);
      intakeSubsystem.runIntakeAtSpd(0);
    }
    
    if ((auxButton_A.getAsBoolean() == true) || (intakeSubsystem.getBallAdvPstn1() == true) ||
        (intakeSubsystem.getBallAdvPstn2() == true)) {
      shooterShutOffTimer.reset();
    }

    SmartDashboard.putNumber("Shooter Speed: ", shooterSubsystem.getSpd());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.runAdvanceAtSpd(0);
    intakeSubsystem.runIntakeAtSpd(0);
    shooterSubsystem.pidShooterSpd(false);
    shooterShutOffTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean condMet = ((auxButton_A.getAsBoolean() == false) || (shooterShutOffTimer.get() >= K_SHOT.KeSHOT_t_PostLaunchRunTime));
    return condMet;
  }
}
