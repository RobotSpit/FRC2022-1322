// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.calibrations.K_INTK;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.controlState;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ManualIntake extends CommandBase {


  private IntakeSubsystem intakeSubsystem;
  private XboxController auxStick;
  private Timer safetyTmr = new Timer();
  private int instrUpdCnt;



  /** Creates a new ManualIntake. */
  public ManualIntake(IntakeSubsystem intakeSubsystem, XboxController auxStick) {
    this.intakeSubsystem = intakeSubsystem;
    this.auxStick = auxStick;
    addRequirements(this.intakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.runIntakeAtSpd(K_INTK.KeINTK_n_TgtIntakeCmdFeed);
    safetyTmr.reset();
    safetyTmr.stop();
    intakeSubsystem.setBallIntakeCtrlSt(controlState.Init);
    instrUpdCnt = (int)0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if(intakeSubsystem.getBallAdvPstn2Filt() == true) {
    //   intakeSubsystem.getAdvanceMtr().set(ControlMode.PercentOutput, .5);
    // }
  
    /*****************************/  
    /* Ball Intake State Machine */
    /*****************************/
    switch (intakeSubsystem.getBallIntakeCtrlSt()) {
      case Init: {

        if((intakeSubsystem.getBallCapturedPstn1() == false) && (intakeSubsystem.getBallCapturedPstn2() == false)) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.SeekBall1);
        }
        else if((intakeSubsystem.getBallCapturedPstn1() == true) && (intakeSubsystem.getBallCapturedPstn2() == true)) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.HoldBall2);
          safetyTmr.start();
        }
        else {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.HoldBall1);
          intakeSubsystem.raiseIntakeArms();
        }
  
        break;
      }
  
      case SeekBall1: {
        if (intakeSubsystem.getBallArmArrayFilt() == true) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.GrabBall1);
          intakeSubsystem.lowerIntakeArms();
          intakeSubsystem.setBallCaptureInProgress(true);
        }
  
        break;
      }
  
      case GrabBall1: {
        if (intakeSubsystem.getBallAdvPstn1Filt() == true) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.HoldBall1);
          intakeSubsystem.runAdvanceAtSpd(K_INTK.KeINTK_n_TgtAdvanceCmdFeed);
          intakeSubsystem.raiseIntakeArms();
          intakeSubsystem.setBallCaptureInProgress(false);
        }  
  
        break;
      }
      
      case HoldBall1: {
        if (intakeSubsystem.getBallAdvPstn2Filt() == true) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.SeekBall2);
          intakeSubsystem.runAdvanceAtSpd(0);
        }
  
        break;
      }
  
      case SeekBall2: {
        if (intakeSubsystem.getBallArmArrayFilt() == true) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.GrabBall2);
          intakeSubsystem.lowerIntakeArms();
          intakeSubsystem.setBallCaptureInProgress(true);
        }
  
        break;
      }
  
      case GrabBall2: {
        if (intakeSubsystem.getBallAdvPstn1() == true) {
          intakeSubsystem.runIntakeAtSpd(K_INTK.KeINTK_n_TgtIntakeCmdFeed);
          intakeSubsystem.setBallCaptureInProgress(false);
        }  

        if ((intakeSubsystem.getBallAdvPstn1Filt() == true) &&
            (intakeSubsystem.getBallAdvPstn2Filt() == true)) {
          intakeSubsystem.setBallIntakeCtrlSt(controlState.HoldBall2); 
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
 
    if (instrUpdCnt == (int)0) {
      SmartDashboard.putNumber("IntakeArmSpd: ",      intakeSubsystem.getIntakeSpd());
      SmartDashboard.putNumber("IntakeAdvSpd: ",      intakeSubsystem.getAdvanceSpd());
    }
    instrUpdCnt++;
    if (instrUpdCnt >= (int)10)
      instrUpdCnt = (int) 0;

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.runIntakeAtSpd(0);
    intakeSubsystem.runAdvanceAtSpd(0);
    safetyTmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean condMet1 = auxStick.getRightTriggerAxis() < K_INTK.KeINTK_r_IntakeMtrTriggerLvlDsbl;
    boolean condMet2 = (safetyTmr.get() >= K_INTK.KeINTK_t_IntakeFullOverrideTmeOut);
    return condMet1 | condMet2;
  }
}
