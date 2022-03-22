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
    public static final boolean KeSHOT_b_DebugEnbl = true;



  /**************************************************/
  /*  Shooter System General System Calibrations    */
 	/**************************************************/	 	

    /** KeSHOT_t_IntakeArmRaiseDlyShoot: Shooter System - Delay
     * Time Prior to Commanding to Raise the Intake Arms after
     * the Balls are no longer detected at the neither Advance
     * Position 1 Nor Position 2 when in the Ball Shoot Command.
     * Allows for some Debounce Filtering.
     */
    public static final double KeSHOT_t_IntakeArmRaiseDlyShoot = 0.100;



  /**************************************************/
  /*  Shooter System Launch Calibrations            */
 	/**************************************************/


	  /** KeSHOT_n_AtTgtDB: Deadband Speed around the
     * Commanded Target that it will be assumed that
     * the Shooter Motor is at the Commanded Target.. 
     */
    public static final double KeSHOT_n_AtTgtDB = 300;

	  /** KeSHOT_n_AtTgtDB_Hystersis: Deadband Speed 
     * Hystersis to determine when to exit the Deadband
     * once entered to that Intake Motor jitter does not
     * occur.
     */
    public static final double KeSHOT_n_AtTgtDB_Hystersis = 500;



	  /** KeSHOT_n_TgtLaunchCmdLoGoal: Shooter System Control 
     * Commmanded Shooter Speed during Tele-Op. 
     */
    public static final double KeSHOT_n_TgtLaunchCmdLoGoal = 4500;


	  /** KeSHOT_n_TgtLaunchCmdLoGoal: Shooter System Control 
     * Commmanded Shooter Speed during Tele-Op. 
     */
    public static final double KeSHOT_n_TgtLaunchCmdLoGoalAuto = 4500;




	  /** KtSHOT_n_ShooterSpdCmdTgtHi: Shooter System Target
     * Motor Speed to as a function of distance to the goal
     * for the high goal. 
     */
    public static final float[] KtSHOT_n_ShooterSpdCmdTgtHi =  
    {  7400,    //  30 
       7800,    //  40
       8000,    //  50
       8700,    //  55
       9100,    //  60
       9500,    //  70
      10000,    //  80
      11000,    //  90
      12000,    // 100
      13000,    // 110
      14000,    // 120
      14500};   // 130

	  /** KnSHOT_l_LaunchServoAxisHi: Shooter System Servo Percent
     * Command Target Distance Axis for High Goal. 
     */
    public static final float KnSHOT_l_LaunchServoAxisHi[] =
    {   30,  //  1
        40,  //  2
        50,  //  3
        55,  //  4
        60,  //  5
        70,  //  6
        80,  //  7
        90,  //  8
       100,  //  9
       110,  // 10
       120,  // 11
       130}; // 12



    /** KeSHOT_t_PostLaunchRunTime: The amount of time
     * to keep the Shooter Motors Running after the balls
     * have left the Ball Feed Advance Positions (assuming
     * the Controller Button is still not being held). 
     */
    public static final double KeSHOT_t_PostLaunchRunTime = 2.0;



    
  /*************************************************/
  /*  Shooter System Control PID Coefficients      */
 	/*************************************************/	 	
    
	  /** KeSHOT_K_Prop: Shooter System Control Proporational
     * Control Gain. 
     */
    public static final double KeSHOT_K_Prop = 0.062;


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
    public static final double KeSHOT_K_FdFwd = 0.0475;

	  /** KeSHOT_r_IntglErrMaxEnbl: Shooter System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral,correction will applied.  Error Signal must
     * be within band (+/- postive value). (revs)
     */
    public static final double KeSHOT_r_IntglErrMaxEnbl = 0.1;



}
