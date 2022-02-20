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
public class K_BALL {



  /***************************************************/
  /*  Ball Feed Control Configuration Calibrations */
 	/***************************************************/	 	

	  /** KeBALL_b_DebugEnbl: Ball Intake System Enable
     *  Calibration to send data to dashbord to debug.
     */
    public static final boolean KeBALL_b_DebugEnbl = false;



  /****************************************************/
  /*  Ball Feed Control General System Calibrations */
 	/****************************************************/	 	




  /**************************************************/
  /*  Ball Feed Motor Speed Calibrations            */
 	/**************************************************/   

	  /** KeBALL_n_TgtIntakeCmd: Ball Feed System Control 
     * Commmanded Intake Motor Speed in RPM. 
     */
    public static final double KeBALL_n_TgtIntakeCmd = 4000;


	  /** KeBALL_n_TgtAdvanceCmd: Ball Feed System Control 
     * Commmanded Advance Motor Speed in RPM. 
     */
    public static final double KeBALL_n_TgtAdvanceCmd = 3000;




  /****************************************************/
  /*  Ball Intake Control PID Coefficients            */
 	/****************************************************/	 	
    
	  /** KeBALL_K_Prop: Ball Intake System Proporational 
     * Control Gain. 
     */
    public static final double KeBALL_K_InProp = 2.0;


	  /** KeBALL_K_Intgl: Ball Intake Intake System
     * Integral Control Gain. 
     */
    public static final double KeBALL_K_InIntgl = 0.001;


	  /** KeBALL_K_Deriv: Ball Intake Intake System
     * Derivative Control Gain. 
     */
    public static final double KeBALL_K_InDeriv = 0.0;


	  /** KeBALL_K_FdFwd: Ball Intake Intake System Feed
     * Fowrward Control Gain. 
     */
    public static final double KeBALL_K_InFdFwd = 0.0;


	  /** KeBALL_r_IntglErrMaxEnbl: Ball Intake System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral correction will applied.  Error Signal must 
     * be within band (+/- postive value). (revs)
     */
    public static final double KeBALL_r_InIntglErrMaxEnbl = 0.1;




  /****************************************************/
  /*  Ball Advance Control PID Coefficients           */
 	/****************************************************/	 	
    
	  /** KeBALL_K_Prop: Ball Advance System Proporational 
     * Control Gain. 
     */
    public static final double KeBALL_K_AdvProp = 2.0;


	  /** KeBALL_K_Intgl: Ball Advance Intake System
     * Integral Control Gain. 
     */
    public static final double KeBALL_K_AdvIntgl = 0.001;


	  /** KeBALL_K_Deriv: Ball Advance Intake System
     * Derivative Control Gain. 
     */
    public static final double KeBALL_K_AdvDeriv = 0.0;


	  /** KeBALL_K_FdFwd: Ball Advance Intake System Feed
     * Fowrward Control Gain. 
     */
    public static final double KeBALL_K_AdvFdFwd = 0.0;


	  /** KeBALL_r_IntglErrMaxEnbl: Ball Advance System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral correction will applied.  Error Signal must 
     * be within band (+/- postive value). (revs)
     */
    public static final double KeBALL_r_AdvIntglErrMaxEnbl = 0.1;





}
