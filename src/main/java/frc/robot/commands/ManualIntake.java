// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.calibrations.K_INTK;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.controlState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ManualIntake extends CommandBase {


  private IntakeSubsystem intakeSubsystem;
  private XboxController auxStick;
  private Timer detectArmTmr = new Timer();  
  private Timer detectAdv1Tmr = new Timer();
  private Timer detectAdv2Tmr = new Timer();
  private Timer safetyTmr = new Timer();



  /** Creates a new ManualIntake. */
  public ManualIntake(IntakeSubsystem intakeSubsystem, XboxController auxStick) {
    //  addRequirements(intakeSubsystem, auxStick);
    this.intakeSubsystem = intakeSubsystem;
    this.auxStick = auxStick;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.runIntakeAtSpd(K_INTK.KeINTK_n_TgtIntakeCmdFeed);
    detectArmTmr.reset();
    detectArmTmr.stop();
    detectAdv1Tmr.reset();
    detectAdv1Tmr.stop();
    detectAdv2Tmr.reset();
    detectAdv2Tmr.stop();
    safetyTmr.reset();
    safetyTmr.stop();
    intakeSubsystem.setBallIntakeCtrlSt(controlState.Init);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  
    if (intakeSubsystem.detectBallAtArm()) {
      detectArmTmr.start();
    }  else {
      detectArmTmr.stop();
      detectArmTmr.reset();
    }
  
    if (intakeSubsystem.detectBallAdvance1()) {
      detectAdv1Tmr.start();
    }  else {
      detectAdv1Tmr.stop();
      detectAdv1Tmr.reset();
    }
  
    if (intakeSubsystem.detectBallAdvance2()) {
      detectAdv2Tmr.start();
    }  else {
      detectAdv2Tmr.stop();
      detectAdv2Tmr.reset();
    }
  
  
  
    /*****************************/  
    /* Ball Intake State Machine */
    /*****************************/
    switch (intakeSubsystem.getBallIntakeCtrlSt()) {
      case Init: {
        if (intakeSubsystem.detectBallAdvance1() == false)
          intakeSubsystem.setBallCapturedPstn1(false);
        if (intakeSubsystem.detectBallAdvance2() == false)
          intakeSubsystem.setBallCapturedPstn2(false); 


        if((intakeSubsystem.getBallCapturedPstn1() == false) && (intakeSubsystem.getBallCapturedPstn2() == false)) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.SeekBall1);
        }
        if((intakeSubsystem.getBallCapturedPstn1() == true) && (intakeSubsystem.getBallCapturedPstn2() == true)) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.HoldBall2);
          safetyTmr.start();
        }
        else {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.HoldBall1);
          intakeSubsystem.releaseIntakeArms();
        }
  
        break;
      }
  
      case SeekBall1: {
        if (detectArmTmr.get() >= K_INTK.KeINTK_t_IntakeDetectTme) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.GrabBall1);
          intakeSubsystem.closeIntakeArms();
          intakeSubsystem.setBallCaptureInProgress(true);
        }
  
        break;
      }
  
      case GrabBall1: {
        if (intakeSubsystem.detectBallAdvance1() == true) {
          intakeSubsystem.releaseIntakeArms();
          intakeSubsystem.setBallCaptureInProgress(false);
        }
  
        if (detectAdv1Tmr.get() >= K_INTK.KeINTK_t_IntakeDetectTme) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.HoldBall1);
        }
  
        break;
      }
      
      case HoldBall1: {
        if (detectAdv2Tmr.get() >= K_INTK.KeINTK_t_IntakeDetectTme) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.SeekBall2);
          intakeSubsystem.setBallCapturedPstn2(true);        
        }
  
        break;
      }
  
      case SeekBall2: {
        if (detectArmTmr.get() >= K_INTK.KeINTK_t_IntakeDetectTme) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.GrabBall2);
          intakeSubsystem.closeIntakeArms();
          intakeSubsystem.setBallCaptureInProgress(true);
        }
  
        break;
      }
  
      case GrabBall2: {
        if ((detectAdv1Tmr.get() >= K_INTK.KeINTK_t_IntakeDetectTme) &&
            (detectAdv2Tmr.get() >= K_INTK.KeINTK_t_IntakeDetectTme)) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.HoldBall2);
          intakeSubsystem.setBallCapturedPstn1(true);  
          safetyTmr.start();      
        }
  
        break;
      }
      
      case HoldBall2: {        
        /* Do nothing - Hold Balls until Shooter Fires */
  
        break;
      }
        
      default: break;
    }
   
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.runIntakeAtSpd(0);
    intakeSubsystem.runAdvanceAtSpd(0);
    detectArmTmr.stop();
    detectAdv1Tmr.stop();
    detectAdv2Tmr.stop();
    safetyTmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean condMet1 = (auxStick.getRightBumperReleased());
    boolean condMet2 = (safetyTmr.get() >= K_INTK.KeINTK_t_IntakeFullOverrideTmeOut);
    return condMet1 || condMet2;
  }
}
