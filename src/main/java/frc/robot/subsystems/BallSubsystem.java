package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.calibrations.K_BALL;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallSubsystem extends SubsystemBase {

public enum slctArm {
  Left,
  Right,
  Front,
  Rear;
}




  private WPI_TalonFX BallAdvanceMotor = new WPI_TalonFX(Constants.BALL_MTR_ADVANCE, "rio");

  private TalonSRX BallIntakeMotor = new TalonSRX(Constants.BALL_MTR_INTAKE);

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

  private DigitalInput BallAdvanceIntake = new DigitalInput(Constants.SW_BALL_ADVANCE_IN);
  private DigitalInput BallAdvanceOuttake = new DigitalInput(Constants.SW_BALL_ADVANCE_OUT);




  /********************************/
  /* BallSubsystem Constructor    */
  /********************************/
  public BallSubsystem() {


 /*****************************************************************/
 /* Ball Intake Motor Controller Configurations                   */
 /*****************************************************************/    
  BallIntakeMotor.configFactoryDefault();
  BallIntakeMotor.setSensorPhase(false);
  BallIntakeMotor.setInverted(false);	
  BallIntakeMotor.setNeutralMode(NeutralMode.Brake);

  BallIntakeMotor.config_kP(0, K_BALL.KeBALL_K_InProp);
  BallIntakeMotor.config_kI(0, K_BALL.KeBALL_K_InIntgl);
  BallIntakeMotor.config_kD(0, K_BALL.KeBALL_K_InDeriv);
  BallIntakeMotor.config_IntegralZone(0, K_BALL.KeBALL_r_InIntglErrMaxEnbl);
  BallIntakeMotor.config_kF(0, K_BALL.KeBALL_K_InFdFwd);


  /*****************************************************************/
  /* Ball Advance Motor Controller Configurations                  */
  /*****************************************************************/  
  BallAdvanceMotor.configFactoryDefault();
  BallAdvanceMotor.setSensorPhase(false);
  BallAdvanceMotor.setInverted(false);	
  BallAdvanceMotor.setNeutralMode(NeutralMode.Brake);

  BallAdvanceMotor.config_kP(0, K_BALL.KeBALL_K_AdvProp);
  BallAdvanceMotor.config_kI(0, K_BALL.KeBALL_K_AdvIntgl);
  BallAdvanceMotor.config_kD(0, K_BALL.KeBALL_K_AdvDeriv);
  BallAdvanceMotor.config_IntegralZone(0, K_BALL.KeBALL_r_AdvIntglErrMaxEnbl);
  BallAdvanceMotor.config_kF(0, K_BALL.KeBALL_K_AdvFdFwd);



  }



  /**
   * Method: getIntakeMtr - Shooter Drive System - Gets the Master Ball Feed Intake Object 
   * @return ShooterMotor[Master]; (WPI_TalonFX: Ball Feed Intake Motor Object)
   */  
  public TalonSRX getIntakeMtr() {
    return BallIntakeMotor;
  }

  public double getIntakeSpd(){
    return getIntakeMtr().getSelectedSensorVelocity();
  }

  public void runIntakeAtSpd(double speed) {
    getIntakeMtr().set(ControlMode.Velocity,speed);
  }

  public void pidIntakeSpd(boolean activate){
    if (activate == true) {
      getIntakeMtr().set(ControlMode.Velocity,K_BALL.KeBALL_n_TgtIntakeCmd);
    } else {
      getIntakeMtr().set(ControlMode.Velocity,0);
    }
  }








  /**
   * Method: getAdvanceMtr - Shooter Drive System - Gets the Master Ball Feed Advance Object 
   * @return ShooterMotor[Master]; (WPI_TalonFX: Ball Feed Advance Motor Object)
   */  
  public WPI_TalonFX getAdvanceMtr() {
    return BallAdvanceMotor;
  }

  public double getAdvanceSpd(){
    return getAdvanceMtr().getSelectedSensorVelocity();
  }

  public void runAdvanceAtSpd(double speed) {
    getAdvanceMtr().set(TalonFXControlMode.Velocity,speed);
  }

  public void pidAdvanceSpd(boolean activate){
    if (activate == true) {
      getAdvanceMtr().set(TalonFXControlMode.Velocity,K_BALL.KeBALL_n_TgtAdvanceCmd);
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



  public boolean detectBallAdvIntake() {
    return (BallAdvanceIntake.get());
  }



  public boolean detectBallAdvOuttake() {
    return (BallAdvanceOuttake.get());
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
