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
public class K_INTK {



  /***************************************************/
  /*  Ball Intake Control Configuration Calibrations */
 	/***************************************************/	 	

	  /** KeINTK_b_DebugEnbl: Ball Intake System Enable
     *  Calibration to send data to dashbord to debug.
     */
    public static final boolean KeINTK_b_DebugEnbl = false;



  /****************************************************/
  /*  Ball Intake Control General System Calibrations */
 	/****************************************************/	 	

    /** KeINTK_r_IntakeMtrTriggerLvlEnbl: Ball Intake System 
     * Aux Controller Right Trigger Postion (0 to 1) at or
     * above that the Ball Intake System will be enabled. 
     */
    public static final double KeINTK_r_IntakeMtrTriggerLvlEnbl = 0.5;

    
    /** KeINTK_r_IntakeMtrTriggerLvlDsbl: Ball Intake System 
     * Aux Controller Right Trigger Postion (0 to 1) below
     * above that the Ball Intake System will be disabled if
     * it is presently active. 
     */
    public static final double KeINTK_r_IntakeMtrTriggerLvlDsbl = 0.3;


    /** KeINTK_t_IntakeArmDtctTme: Ball Intake System Ball 
     * present minimun detection time for Intake Arms.
     */
    public static final double KeINTK_t_IntakeArmDtctTme = 0.060;

    /** KeINTK_t_IntakeAdv1DtctTme: Ball Intake System Ball 
     * present minimun detection time for Intake Advance
     * Position 1.
     */
    public static final double KeINTK_t_IntakeAdv1DtctTme = 0.060;

    /** KeINTK_t_IntakeAdv1DtctTme: Ball Intake System Ball 
     * present minimun capture time for Intake Advance
     * Position 1.
     */
    public static final double KeINTK_t_IntakeAdv1CaptTme = 0.060;

    /** KeINTK_t_IntakeAdv2DtctTme: Ball Intake System Ball 
     * present minimun detection time for Intake Advance
     * Position 2.
     */
    public static final double KeINTK_t_IntakeAdv2DtctTme = 0.060;

    /** KeINTK_t_IntakeAdv2DtctTme: Ball Intake System Ball 
     * present minimun capture time for Intake Advance
     * Position 2.
     */
    public static final double KeINTK_t_IntakeAdv2CaptTme = 0.060;



    /** KeINTK_t_IntakeFullOverrideTmeOut: Ball Intake
     * System Ball Full Override Time Out.  If the Operator
     * is Requesting to Run the Intake Motors without Shooting
     * but Two Balls are detected in the Intake System, time
     * out the Intake and Advance Motors after this time. 
     */
    public static final double KeINTK_t_IntakeFullOverrideTmeOut = 0.500;


    /** KeINTK_t_IntakeArmRaiseLowerDly: Ball Intake System Delay
     * Time To Keep Command Active before terminating when requesting
     * to Raise or Lower the Intake Arms.
     */
    public static final double KeINTK_t_IntakeArmRaiseLowerDly = 0.025;



  /**************************************************/
  /*  Ball Feed Motor Speed Calibrations            */
 	/**************************************************/   

   	/** KeINTK_n_TgtIntakeCmdFeed: Ball Intake System Control 
     * Commmanded Intake Motor Speed in RPM during Ball Feed. 
     */
    public static final double KeINTK_n_TgtIntakeCmdFeed = 4000;


	  /** KeINTK_n_TgtAdvanceCmdFeed: Ball Intake System Control 
     * Commmanded Advance Motor Speed in RPM during Ball Feed. 
     */
    public static final double KeINTK_n_TgtAdvanceCmdFeed = 1500;


	  /** KeINTK_n_TgtIntakeCmdShoot: Ball Intake System Control 
     * Commmanded Intake Motor Speed in RPM during Ball Shoot. 
     */
    public static final double KeINTK_n_TgtIntakeCmdShoot = 5000;


	  /** KeINTK_n_TgtAdvanceCmdShoot: Ball Intake System Control 
     * Commmanded Advance Motor Speed in RPM during Ball Shoot. 
     */
    public static final double KeINTK_n_TgtAdvanceCmdShoot = 5000;



    
  /****************************************************/
  /*  Ball Intake Control PID Coefficients            */
 	/****************************************************/	 	
    
	  /** KeINTK_K_Prop: Ball Intake System Proporational 
     * Control Gain. 
     */
    public static final double KeINTK_K_InProp = 0.1;


	  /** KeINTK_K_Intgl: Ball Intake System Integral 
     * Control Gain. 
     */
    public static final double KeINTK_K_InIntgl = 0;


	  /** KeINTK_K_Deriv: Ball Intake System Derivative
     * Control Gain. 
     */
    public static final double KeINTK_K_InDeriv = 0;


	  /** KeINTK_K_FdFwd: Ball Intake System Feed Fowrward
     * Control Gain. 
     */
    public static final double KeINTK_K_InFdFwd = 0.1;


	  /** KeINTK_r_IntglErrMaxEnbl: Ball Intake System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral correction will applied.  Error Signal must 
     * be within band (+/- postive value). (revs)
     */
    public static final double KeINTK_r_InIntglErrMaxEnbl = 0.1;




  /****************************************************/
  /*  Ball Advance Control PID Coefficients           */
 	/****************************************************/	 	
    
	  /** KeINTK_K_Prop: Ball Advance System Proporational 
     * Control Gain. 
     */
    public static final double KeINTK_K_AdvProp = 0.1;


	  /** KeINTK_K_Intgl: Ball Advance Intake System
     * Integral Control Gain. 
     */
    public static final double KeINTK_K_AdvIntgl = 0;


	  /** KeINTK_K_Deriv: Ball Advance Intake System
     * Derivative Control Gain. 
     */
    public static final double KeINTK_K_AdvDeriv = 0;


	  /** KeINTK_K_FdFwd: Ball Advance Intake System Feed
     * Fowrward Control Gain. 
     */
    public static final double KeINTK_K_AdvFdFwd = 0.1;


	  /** KeINTK_r_IntglErrMaxEnbl: Ball Advance System Control
     * Maximum Error Signal Threshold (absolute value) that
     * Integral correction will applied.  Error Signal must 
     * be within band (+/- postive value). (revs)
     */
    public static final double KeINTK_r_AdvIntglErrMaxEnbl = 0.1;



}
