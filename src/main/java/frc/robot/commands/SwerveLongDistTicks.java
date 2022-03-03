// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveLongDistTicks extends CommandBase {
  private SwerveDrivetrain swerveDrivetrain;
  private double distInches;
  private boolean isDirctnFwd;
  private int caddyIndexA;
  private int caddyIndexB;
  private double zeroEncdrCntA;
  private double zeroEncdrCntB;

  /** Creates a new DriveLongDistTicks. */
  public SwerveLongDistTicks(SwerveDrivetrain swerveDrivetrain,double distInches, boolean isDirctnFwd) {
    this.swerveDrivetrain = swerveDrivetrain;
    this.distInches = distInches;
    this.isDirctnFwd = isDirctnFwd; 
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
