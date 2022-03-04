// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.calibrations.old.K_SWRV;
import frc.robot.subsystems.SwerveDrivetrain;

public class CA_SwerveLongDistEncdr extends CommandBase {
  private SwerveDrivetrain swerveDrivetrain;
  private Timer delayTmr;
  private double tgtDistInches;
  private boolean isDirctnFwd;
  private int caddyIndexA;
  private int caddyIndexB;
  private double zeroEncdrCntA;
  private double zeroEncdrCntB;
  private double distTravelledA;
  private double distTravelledB;
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;



  /** Creates a new CA_SwerveLongDistEncdr. */
  public CA_SwerveLongDistEncdr(SwerveDrivetrain swerveDrivetrain,double tgtDistInches, boolean isDirctnFwd) {
    this.swerveDrivetrain = swerveDrivetrain;
    this.tgtDistInches = tgtDistInches;
    this.isDirctnFwd = isDirctnFwd;
    delayTmr = new Timer();
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delayTmr.reset();
    delayTmr.start();
    caddyIndexA = 0; 
    caddyIndexB = 1;
    zeroEncdrCntA = swerveDrivetrain.getDrvCaddyEncdrPstn(caddyIndexA);
    zeroEncdrCntB = swerveDrivetrain.getDrvCaddyEncdrPstn(caddyIndexB);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distTravelledA = swerveDrivetrain.getDrvDistTravelled(caddyIndexA, zeroEncdrCntA);
    distTravelledB = swerveDrivetrain.getDrvDistTravelled(caddyIndexB, zeroEncdrCntB);

    if (delayTmr.get() >= 0.100) {
      double yAxis = 0.4;
      double xAxis = 0.0;
      double rAxis = 0.0;

      if (!isDirctnFwd) {
        yAxis = -yAxis;
      }

      translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveDrivetrain.MAX_SPEED);
      rotation = rAxis * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
      swerveDrivetrain.drive(translation, rotation, fieldRelative, openLoop);
    }

    if (K_SWRV.KeSWRV_b_DebugEnbl == true) {
      SmartDashboard.putNumber("Encdr Cnt Zero A: ",   (zeroEncdrCntA));
      SmartDashboard.putNumber("Encdr Cnt Zero B: ",   (zeroEncdrCntB));
      SmartDashboard.putNumber("Encdr Cnt Curr A: ",   (swerveDrivetrain.getDrvCaddyEncdrPstn(caddyIndexA)));
      SmartDashboard.putNumber("Encdr Cnt Curr B: ",   (swerveDrivetrain.getDrvCaddyEncdrPstn(caddyIndexB)));
      SmartDashboard.putNumber("Dist Travelled A: ",   (distTravelledA));
      SmartDashboard.putNumber("Dist Travelled B: ",   (distTravelledB));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrain.stopSwerveDriveMotors();
    delayTmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean condMetA = Math.abs(distTravelledA) >= tgtDistInches;
    boolean condMetB = Math.abs(distTravelledB) >= tgtDistInches;
    return (condMetA && condMetB);
  }
}
