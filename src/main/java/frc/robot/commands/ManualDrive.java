/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.calibrations.K_SWRV;
import frc.robot.subsystems.RFSLIB;
import frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualDrive extends CommandBase {
  /**
   * Command: ManualDrive Command t    driveSubsystem.zeroGyro();o Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  private final SwerveDriveSubsystem swerveSubsystem;
  private final XboxController driverStick;
  private final RFSLIB rfsLIB = new RFSLIB();
  private double Xe_r_LongPwr, Xe_r_LatPwr, Xe_r_RotPwr;


  public ManualDrive(SwerveDriveSubsystem swerveSubsystem, XboxController driverStick) {
      this.swerveSubsystem = swerveSubsystem;
      this.driverStick = driverStick;
      swerveSubsystem.resetDrvEncdrs();
      addRequirements(this.swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Xe_r_LongPwr = -driverStick.getLeftY();
    Xe_r_LatPwr  =  driverStick.getLeftX();
    Xe_r_RotPwr  =  driverStick.getRightX();

    Xe_r_LongPwr = rfsLIB.ApplyDB_Scld(Xe_r_LongPwr, K_SWRV.KeSWRV_r_CntlrDeadBandThrsh, 1.0);
    Xe_r_LatPwr  = rfsLIB.ApplyDB_Scld(Xe_r_LatPwr, K_SWRV.KeSWRV_r_CntlrDeadBandThrsh, 1.0);
    Xe_r_RotPwr  = rfsLIB.ApplyDB_Scld(Xe_r_RotPwr, K_SWRV.KeSWRV_r_CntlrDeadBandThrsh, 1.0);
  
    swerveSubsystem.HolonomicDrv(Xe_r_LongPwr, Xe_r_LatPwr, Xe_r_RotPwr, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      swerveSubsystem.stopDrvMtrs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return (false);
  }

}
