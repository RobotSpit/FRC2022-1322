package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.calibrations.K_SHOT;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  public enum slctMtr {
    Left,
    Right;
 }




 private WPI_TalonFX[] ShooterMotor = new WPI_TalonFX[] {
  new WPI_TalonFX(Constants.SHOOTER_MTR_LT, "rio"),
  new WPI_TalonFX(Constants.SHOOTER_MTR_RT, "rio")  
};








  /********************************/
  /* ShooterSubsystem Constructor */
  /********************************/
  public ShooterSubsystem() {




    /*****************************************************************/
    /* Shooter Motor Controller Configurations                       */
    /*****************************************************************/
    for (int i = 0; i < 2; i++) {
      ShooterMotor[i].configFactoryDefault();
      ShooterMotor[i].setSensorPhase(false);
      ShooterMotor[i].setNeutralMode(NeutralMode.Brake);
      ShooterMotor[i].config_kP(0, K_SHOT.KeSHOT_K_Prop);
      ShooterMotor[i].config_kI(0, K_SHOT.KeSHOT_K_Intgl);
      ShooterMotor[i].config_kD(0, K_SHOT.KeSHOT_K_Deriv);
      ShooterMotor[i].config_IntegralZone(0, K_SHOT.KeSHOT_r_IntglErrMaxEnbl);
      ShooterMotor[i].config_kF(0, K_SHOT.KeSHOT_K_FdFwd);      
    }
	
    ShooterMotor[Constants.SHOOTER_MTR_LT].setInverted(false);			
    ShooterMotor[Constants.SHOOTER_MTR_RT].setInverted(true);			
    ShooterMotor[Constants.SHOOTER_MTR_RT].follow(ShooterMotor[Constants.SHOOTER_MTR_LT]);

    

  }




  /**
   * Method: getShooterMtr - Shooter Drive System - Gets the Master Shooter Motor Object 
   * @return ShooterMotor[Master]; (WPI_TalonFX: Shooter Motor Object)
   */  
  public WPI_TalonFX getShooterMtr() {
    return ShooterMotor[Constants.SHOOTER_MTR_LT];
  }

  public double getSpd(){
    return getShooterMtr().getSelectedSensorVelocity();
  }

  public boolean isShooterAtSpd(){
    return Math.abs(K_SHOT.KeSHOT_n_TgtLaunchCmd - getSpd()) < K_SHOT.KeSHOT_n_AtTgtDB;
  }

  public void runShooterAtSpd(double speed) {
    getShooterMtr().set(TalonFXControlMode.Velocity,speed);
  }

  public void pidShooterSpd(boolean activate){
    if (activate == true) {
      getShooterMtr().set(TalonFXControlMode.Velocity,K_SHOT.KeSHOT_n_TgtLaunchCmd);
    } else {
      getShooterMtr().set(TalonFXControlMode.Velocity,0);
    }
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
