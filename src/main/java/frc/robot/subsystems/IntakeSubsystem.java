package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.calibrations.K_INTK;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public enum slctArm {
    Left,
    Right,
    Front,
    Rear
  }

  public enum controlState {
    Init,
    SeekBall1,
    GrabBall1,
    HoldBall1,
    SeekBall2,
    GrabBall2,
    HoldBall2
  }



  private WPI_TalonFX AdvanceMotor = new WPI_TalonFX(Constants.BALL_MTR_ADVANCE, "rio");

  private TalonSRX IntakeMotor = new TalonSRX(Constants.BALL_MTR_INTAKE);

  private Solenoid[] BallIntakeArm = new Solenoid[] {
    new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEU_BALL_INTAKE_LT),
    new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEU_BALL_INTAKE_RR),
    new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEU_BALL_INTAKE_RT),
    new Solenoid(PneumaticsModuleType.REVPH, Constants.PNEU_BALL_INTAKE_FT)
  };



  private DigitalInput[] BallArmDetect = new DigitalInput[] {
    new DigitalInput(Constants.SW_BALL_INTAKE_LT),
    new DigitalInput(Constants.SW_BALL_INTAKE_RR),
    new DigitalInput(Constants.SW_BALL_INTAKE_RT),
    new DigitalInput(Constants.SW_BALL_INTAKE_FT)
  };

  private DigitalInput BallAdvance1 = new DigitalInput(Constants.SW_BALL_ADVANCE_1);
  private DigitalInput BallAdvance2 = new DigitalInput(Constants.SW_BALL_ADVANCE_2);

  private boolean ballCaptureInProgress;
  private boolean ballCapturedPstn1;
  private boolean ballCapturedPstn2;

  private controlState ballIntakeCtrlSt;



  /********************************/
  /* BallSubsystem Constructor    */
  /********************************/
  public IntakeSubsystem() {


 /*****************************************************************/
 /* Ball Intake Motor Controller Configurations                   */
 /*****************************************************************/    
  IntakeMotor.configFactoryDefault();
  IntakeMotor.setSensorPhase(false);
  IntakeMotor.setInverted(false);	
  IntakeMotor.setNeutralMode(NeutralMode.Brake);

  IntakeMotor.config_kP(0, K_INTK.KeINTK_K_InProp);
  IntakeMotor.config_kI(0, K_INTK.KeINTK_K_InIntgl);
  IntakeMotor.config_kD(0, K_INTK.KeINTK_K_InDeriv);
  IntakeMotor.config_IntegralZone(0, K_INTK.KeINTK_r_InIntglErrMaxEnbl);
  IntakeMotor.config_kF(0, K_INTK.KeINTK_K_InFdFwd);


  /*****************************************************************/
  /* Ball Advance Motor Controller Configurations                  */
  /*****************************************************************/  
  AdvanceMotor.configFactoryDefault();
  AdvanceMotor.setSensorPhase(false);
  AdvanceMotor.setInverted(false);	
  AdvanceMotor.setNeutralMode(NeutralMode.Brake);

  AdvanceMotor.config_kP(0, K_INTK.KeINTK_K_AdvProp);
  AdvanceMotor.config_kI(0, K_INTK.KeINTK_K_AdvIntgl);
  AdvanceMotor.config_kD(0, K_INTK.KeINTK_K_AdvDeriv);
  AdvanceMotor.config_IntegralZone(0, K_INTK.KeINTK_r_AdvIntglErrMaxEnbl);
  AdvanceMotor.config_kF(0, K_INTK.KeINTK_K_AdvFdFwd);

  ballCaptureInProgress = false;
  ballCapturedPstn1 = false;
  ballCapturedPstn2 = false;
  ballIntakeCtrlSt = controlState.Init;

  }



  /**
   * Method: getIntakeMtr - Shooter Drive System - Gets the Master Ball Feed Intake Object 
   * @return ShooterMotor[Master]; (WPI_TalonFX: Ball Feed Intake Motor Object)
   */  
  public TalonSRX getIntakeMtr() {
    return IntakeMotor;
  }

  public double getIntakeSpd(){
    return getIntakeMtr().getSelectedSensorVelocity();
  }

  public void runIntakeAtSpd(double speed) {
    getIntakeMtr().set(ControlMode.Velocity,speed);
  }

  public void pidIntakeSpd(boolean activate){
    if (activate == true) {
      getIntakeMtr().set(ControlMode.Velocity,K_INTK.KeINTK_n_TgtIntakeCmdFeed);
    } else {
      getIntakeMtr().set(ControlMode.Velocity,0);
    }
  }




  /**
   * Method: getAdvanceMtr - Shooter Drive System - Gets the Master Ball Feed Advance Object 
   * @return ShooterMotor[Master]; (WPI_TalonFX: Ball Feed Advance Motor Object)
   */  
  public WPI_TalonFX getAdvanceMtr() {
    return AdvanceMotor;
  }

  public double getAdvanceSpd(){
    return getAdvanceMtr().getSelectedSensorVelocity();
  }

  public void runAdvanceAtSpd(double speed) {
    getAdvanceMtr().set(TalonFXControlMode.Velocity,speed);
  }

  public void pidAdvanceSpd(boolean activate){
    if (activate == true) {
      getAdvanceMtr().set(TalonFXControlMode.Velocity,K_INTK.KeINTK_n_TgtAdvanceCmdShoot);
    } else {
      getAdvanceMtr().set(TalonFXControlMode.Velocity,0);
    }
  }





  public void closeIntakeArms() {
    int i;
    for (i = 0; i < 4; i++) {
      BallIntakeArm[i].set(true);
    }
  }


  public void releaseIntakeArms() {
    int i;
    for (i = 0; i < 4; i++) {
      BallIntakeArm[i].set(false);
    }
  }



  public boolean detectBallAtArm() {
    boolean ballDetected = false;
    int i;
    for (i = 0; i < 4; i++) {
      if (BallArmDetect[i].get() == true) {
        ballDetected = true;
      }
    }
    return ballDetected;
  }



  public boolean detectBallAdvance1() {
    return (BallAdvance1.get());
  }



  public boolean detectBallAdvance2() {
    return (BallAdvance2.get());
  }



  public void setBallCaptureInProgress(boolean captureInProgress) {
    ballCaptureInProgress = captureInProgress;
  }

  public boolean getBallCaptureInProgress() {
    return (ballCaptureInProgress);
  }



  public void setBallCapturedPstn1(boolean ballCaptured) {
    ballCapturedPstn1 = ballCaptured;
  }

  public boolean getBallCapturedPstn1() {
    return (ballCapturedPstn1);
  }



  public void setBallCapturedPstn2(boolean ballCaptured) {
    ballCapturedPstn2 = ballCaptured;
  }

  public boolean getBallCapturedPstn2() {
    return (ballCapturedPstn2);
  }




  public void setBallIntakeCtrlSt(controlState  intakeCtrlSt) {
    ballIntakeCtrlSt = intakeCtrlSt;
  }

  public controlState getBallIntakeCtrlSt() {
    return (ballIntakeCtrlSt);
  }
 




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
