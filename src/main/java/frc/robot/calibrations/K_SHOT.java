/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;


/**
 * Add your docs here.
 */
public class K_SHOT {



  /**************************************************/
  /*  Shooter System Configuration Calibrations     */
 	/**************************************************/	 	

	  /** KeSHOT_b_DebugEnbl: Shooter System Enable
     *  Calibration to send data to dashbord to debug.
     */
    public static final boolean KeSHOT_b_DebugEnbl = false;



  /**************************************************/
  /*  Shooter System General System Calibrations    */
 	/**************************************************/	 	





  /**************************************************/
  /*  Shooter System Launch Calibrations            */
 	/**************************************************/

	  /** KeSHOT_n_ShooterSpd: Shooter System Control 
     * Commmanded Shooter Speed. 
     */
    public static final double KeSHOT_n_TgtLaunchCmd = 3000;


	  /** KeSHOT_n_AtTgtDB: Deadband Speed around the
     * Commanded Target that it will be assumed that
     * the Shooter Motor is at the Commanded Target.. 
     */
    public static final double KeSHOT_n_AtTgtDB = 300;

	  /** KtSHOT_Pct_LaunchServoCmdHi: Shooter System Servo Percent
     * Command to deflect the shooter deflector shield to
     * aim the shooter as a function of distance to the goal
     * for the high goal. 
     */
    public static final float[] KtSHOT_Pct_LaunchServoCmdHi =  
     { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


	  /** KnSHOT_l_LaunchServoAxisHi: Shooter System Servo Percent
     * Command Target Distance Axis for High Goal. 
     */
    public static final float KnSHOT_l_LaunchServoAxisHi[] =
    { 5, 8, 10, 15, 20, 25, 30, 35, 40, 50};


	  /** KtSHOT_Pct_LaunchServoCmdLo: Shooter System Servo Percent
     * Command to deflect the shooter deflector shield to
     * aim the shooter as a function of distance to the goal
     * for the high goal. 
     */
    public static final float KtSHOT_Pct_LaunchServoCmdLo[] =
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


	  /** KnSHOT_l_LaunchServoAxisLo: Shooter System Servo Percent
     * Command Target Distance Axis for High Goal. 
     */
    public static final float KnSHOT_l_LaunchServoAxisLo[] =
    { 2, 5, 8, 10, 12, 15, 18, 20, 22, 25 };


    /** KeSHOT_t_PostLaunchRunTime: The amount of time
     * to keep the Shooter Motors Running after the balls
     * have left the Ball Feed Advance Positions (assuming
     * the Controller Button is still not being held). 
     */
    public static final double KeSHOT_t_PostLaunchRunTime = 3.0;



    
  /*************************************************/
  /*  Shooter System Control PID Coefficients      */
 	/*************************************************/	 	
    
	  /** KeSHOT_K_Prop: Shooter System Control Proporational
     * Control Gain. 
     */
    public static final double KeSHOT_K_Prop = 1.0;


	  /** KeSHOT_K_Intgl: Shooter System Control Integral 
     * Control Gain. 
     */
    public static final double KeSHOT_K_Intgl = 0.0;


	  /** KeSHOT_K_Deriv: Shooter System Control
     * Derivative Control Gain. 
     */
    public static final double KeSHOT_K_Deriv = 0.0;


	  /** KeSHOT_K_FdFwd: Shooter System Control Feed
     * Fowrward Control Gain. 
     */
    public static final double KeSHOT_K_FdFwd = 10.0;


	  /** KeSHOT_r_IntglErrMaxEnbl: Shooter System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral,correction will applied.  Error Signal must
     * be within band (+/- postive value). (revs)
     */
    public static final double KeSHOT_r_IntglErrMaxEnbl = 0.1;



}
