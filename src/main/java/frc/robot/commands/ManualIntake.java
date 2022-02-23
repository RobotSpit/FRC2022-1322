// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ManualIntake extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private XboxController auxStick;
  private  Timer intakeTimer = new Timer();

  /** Creates a new ManualIntake. */
  public ManualIntake(IntakeSubsystem intakeSubsystem, XboxController auxStick) {
    //  addRequirements(intakeSubsystem, auxStick);
    this.intakeSubsystem = intakeSubsystem;
    this.auxStick = auxStick;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.pidIntakeSpd(true);
    intakeTimer.reset();
    intakeTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intakeSubsystem.detectBallAdvance1()) {
      intakeSubsystem.closeIntakeArms();
      intakeTimer.start();
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.pidIntakeSpd(false);
    intakeTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
