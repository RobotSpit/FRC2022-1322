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
    public static final double KeSHOT_n_TgtLaunchCmd = 5000;


	  /** KeSHOT_n_ShooterSpd: Deadband Speed around the
     * Commanded Target that it will be assumed that
     * the Shooter Motor is at the Commanded Target.. 
     */
    public static final double KeSHOT_n_AtTgtDB = 300;





  /*************************************************/
  /*  Shooter System Control PID Coefficients      */
 	/*************************************************/	 	
    
	  /** KeSHOT_K_Prop: Shooter System Control Proporational
     * Control Gain. 
     */
    public static final double KeSHOT_K_Prop = 0.0001;


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
    public static final double KeSHOT_K_FdFwd = 0.000185;


	  /** KeSHOT_r_IntglErrMaxEnbl: Shooter System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral,correction will applied.  Error Signal must
     * be within band (+/- postive value). (revs)
     */
    public static final double KeSHOT_r_IntglErrMaxEnbl = 0.1;



}
