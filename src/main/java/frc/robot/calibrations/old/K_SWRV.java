/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations.old;

/**
 * Add your docs here.
 */
public class K_SWRV {



  /**************************************************/
  /*  Swerve Drive Configuration Calibrations       */
 	/**************************************************/	 	

	  /** KeSWRV_b_DebugEnbl: Swerve Drive System Enable
     *  Calibration to send data to dashbord to debug.
     */
    public static final boolean KeSWRV_b_DebugEnbl = false;


   /** KeSWRV_b_RZL_Enbl: Swerve Drive System - Rotation
    * Zero Offset Learn Algorthm Enable.
     */
    public static final boolean KeSWRV_b_RZL_Enbl = true;

    
	  /** KeSWRV_b_DrvMtrRotDirctnInvertEnbl: Swerve Drive System Enable
     *  Calibration to Enable Drive Motor Direction Inversion.
     */
    public static final boolean KeSWRV_b_DrvMtrRotDirctnInvertEnbl = false;

	  /** KeSWRV_b_DrvMtrRotDirctnInvertInhbRot: Swerve Drive System Enable
     *  Calibration to Inhibit Drive Motor Direction Inversion when 
     *  the Drive System is in Rotate Mode.
     */
    public static final boolean KeSWRV_b_DrvMtrRotDirctnInvertInhbRot = true;



  /**************************************************/
  /*  Swerve Drive General System Calibrations      */
 	/**************************************************/	 	

	  /** KeSWRV_r_CntlrDeadBandThrsh: Swerve Drive System: Normalized
     * Power Dead-Band Threshold that must be met before a X-Box Controller
     * Joystick Input is recognized.  If the Absolute value of the Input
     * is below the Threshold it will be ignored (i.e. set to Zero).
     */
    public static final double KeSWRV_r_CntlrDeadBandThrsh = 0.3;


	  /** KeSWRV_r_DrvRqstOvrrdFwd: Swerve Drive System: Normalized
     * Power Threshold used to so select between Long/Lat Control
     * vs Rotation Control.
     */
    public static final double KeSWRV_r_DrvRqstOvrrdFwd = 0.1;

	  /** KeSWRV_r_DrvRqstOvrrdRot: Swerve Drive System: Normalized
     * Power Threshold used to so select between Long/Lat Control
     * vs Rotation Control.
     */
    public static final double KeSWRV_r_DrvRqstOvrrdRot = 0.1;



  /**************************************************/
  /*  Swerve Drive Design Parameters                */
 	/**************************************************/	 	

	  /** KeSWRV_l_ChassisWhlBase: Swerve Drive System Drive Chassis
     * Effective WheelBase.
     */
    public static final double KeSWRV_l_ChassisWhlBase = 23.5;

	  /** KeSWRV_l_ChassisTrkWdth: Swerve Drive System Drive Chassis
     * Effective TrackWidtch.
     */
    public static final double KeSWRV_l_ChassisTrkWdth = 23.5;



  /**************************************************/
  /*  Swerve Rotation-System Mechanical Parameters  */
 	/**************************************************/	 	

	  /** KeSWRV_Cf_RotMtrEncdrCntsPerRev: Swerve Drive System - Conversion
     * Factor of the Number of Encoder Counts Per One Rotation of the
     * Rotation Control Motor.
     */
    public static final double KeSWRV_Cf_RotMtrEncdrCntsPerRev = 2048;


	  /** KeSWRV_r_RotMtrEncdrToCaddyRat: Swerve Drive System: Number of
     * Rotation Control Motor Encoder Rotations to Swerve Module Rotations
     * Ratio.
     */
    public static final double KeSWRV_r_RotMtrEncdrToCaddyRat = 19;



  /*******************************************************/
  /*  Swerve Drive-System Design Mechanical Parameters   */
 	/*******************************************************/	 	
	
	  /** KeSWRV_Cnt_DrvEncdrCntsPerRev: Swerve Drive System: Number
     * of Encoder Counts Per One Revolution of the Drive Motors
     */
    public static final double KeSWRV_Cnt_DrvEncdrCntsPerRev = 2048;


	  /** KeSWRV_l_DrvWhlDistPerRev: Swerve Drive System: Number of lineal
     * distance travaled per one rotation of Swerve Drive Wheel Revolution
     */
    public static final double KeSWRV_l_DrvWhlDistPerRev = 4.0 * Math.PI; // 12.57142857

    
    /** KeSWRV_r_DrvMtrEncdrToWhlRat: Swerve Drive System: Number of
     * Drive Control Motor Encoder Rotations to Swerve Wheel Rotations
     * Ratio.
     */
    public static final double KeSWRV_r_DrvMtrEncdrToWhlRat = 10;





  /****************************************************/
  /*  Swerve Drive Rotation Control PID Coefficients  */
 	/****************************************************/	 	
    
	  /** KeSWRV_K_RotProp: Swerve Drive System Rotation Control
     * Proporational Control Gain. 
     */
    public static final double KeSWRV_K_RotProp = 0.01;


	  /** KeSWRV_K_RotIntgl: Swerve Drive System Rotation Control
     * Integral Control Gain. 
     */
    public static final double KeSWRV_K_RotIntgl = 0.0;


	  /** KeSWRV_K_RotDeriv: Swerve Drive System Rotation Control
     * Derivative Control Gain. 
     */
    public static final double KeSWRV_K_RotDeriv = 0.0;


	  /** KeSWRV_K_RotFdFwd: Swerve Drive System Rotation Control
     * Feed Fowrward Control Gain. 
     */
    public static final double KeSWRV_K_RotFdFwd = 0.1;


	  /** KeSWRV_r_RotIntglErrMaxEnbl: Swerve Drive System Rotation Control
     * Maximum Error Signal Threshold (absolute value) that Integral
     * correction will applied.  Error Signal must be within band 
     * (+/- postive value). (revs)
     */
    public static final double KeSWRV_r_RotIntglErrMaxEnbl = 0.1;


	  /** KeSWRV_r_RotNormOutMax: Swerve Drive System Rotation Control
     * Normalized Output Power Maximum Limit. This is the Absolute Power
     * Limit so will be the Max Limit in the Postive or Negative direction.
     */
    public static final double KeSWRV_r_RotNormOutMax = 1;


	  /** KeSWRV_r_RotNormOutMin: Swerve Drive System Rotation Control
     * Normalized Output Power Maximum Limit TimeOut (in mSec).
     * A value of 0 mSec will apply no TimeOut function.  
     */
    public static final int KeSWRV_t_RotNormOutMaxTmeOut = 0;
 


  /**************************************************/
  /*  Swerve Drive Drive Control PID Coefficients   */
 	/**************************************************/	 	

	  /** KeSWRV_K_DrvProp: Swerve Drive System Drive Control
     * Proporational Control Gain. 
     */
    public static final double KeSWRV_K_DrvProp = 2.0;


	  /** KeSWRV_K_DrvIntgl: Swerve Drive System Drive Control
     * Integral Control Gain. 
     */
    public static final double KeSWRV_K_DrvIntgl = 0.0;


	  /** KeSWRV_K_DrvDeriv: Swerve Drive System Drv Control
     * Derivative Control Gain. 
     */
    public static final double KeSWRV_K_DrvDeriv = 0.0;


	  /** KeSWRV_K_DrvFdFwd: Swerve Drive System Drive Control
     * Feed Fowrward Control Gain. 
     */
    public static final double KeSWRV_K_DrvFdFwd = 0.0;


	  /** KeSWRV_n_DrvIntglErrMaxEnbl: Swerve Drive System Drive Control
     * Maximum Error Signal Threshold (absolute value) that Integral
     * correction will applied.  Error Signal must be within band 
     * (+/- postive value). (rpm/100ms) 
     */
    public static final int KeSWRV_n_DrvIntglErrMaxEnbl = 0;


	  /** KeSWRV_n_Drv_MM_CruiseVel: Swerve Drive System: This is the
     * peak target velocity that the motion magic curve generator can
     * use for the Drive Control Speed Closed-Loop. (RPM/100msec) 
     */
    public static final int KeSWRV_n_Drv_MM_CruiseVel = 9000;


	  /** KeSWRV_a_Drv_MM_MaxAccel: Swerve Drive System: Sets the Motion
     * Magic Acceleration. This is the target acceleration that the motion
     * magic curve generator can use for the Drive Control Speed
     * Closed-Loop.  (RPM/100msec/sec)
     */
    public static final int KeSWRV_a_Drv_MM_MaxAccel = 13000;


	  /** KeSWRV_r_DrvNormOutMax: Swerve Drive System Drive Control
     * Normalized Output Power Maximum Limit. This is the Absolute Power
     * Limit so will be the Max Limit in the Postive or Negative direction.
     */
    public static final double KeSWRV_r_DrvNormOutMax = 1;


	  /** KeSWRV_r_DrvNormOutMin: Swerve Drive System Drive Control
     * Normalized Output Power Maxmimum Limit TimeOut (in mSec).
     * A value of 0 mSec will apply no TimeOut function. 
     */
    public static final int KeSWRV_t_DrvNormOutMaxTmeOut = 0;
 



  /******************************************/
  /*  Swerve Drive Current Driver Limits    */
 	/******************************************/	 	
 
   /** KeSWRV_I_RotDrvrLmtMaxPri: Swerve Drive System Rotation Control
     * Motor Driver Maximum Current Limit for Primary Driver .
     */
    public static final int KeSWRV_I_RotDrvrLmtMaxPri = 60;

 
   /** KeSWRV_I_RotDrvrLmtMaxSec: Swerve Drive System Rotation Control
     * Motor Driver Maximum Current Limit for Secondary Driver. 
     */
    public static final double KeSWRV_I_RotDrvrLmtMaxSec = 20;

   /** KeSWRV_t_RotCAN_TmeOut: Swerve Drive System Rotation Control
     * Motor Driver CAN Message Time Out Threshold (in Seconds). 
     */
    public static final double KeSWRV_t_RotCAN_TmeOut = 2;



   /** KeSWRV_I_DrvDrvrLmtMaxPri: Swerve Drive System: Drive Control
     * Motor Driver Maximum Current Limit for Primary Driver,
     * Supply Current Limit.
     */
    public static final double KeSWRV_I_DrvDrvrLmtMaxPri = 60;

 
   /** KeSWRV_I_DrvDrvrLmtMaxSec: Swerve Drive System Drive Control
     * Motor Driver Maximum Current Limit for Secondary Driver,
     * Stator Current Limit. 
     */
    public static final double KeSWRV_I_DrvDrvrLmtMaxSec = 60;


   /** KeSWRV_t_DrvCAN_TmeOut: Swerve Drive System Rotation Control
     * Motor Driver CAN Message Time Out Threshold (in Seconds). 
     */
    public static final double KeSWRV_t_DrvCAN_TmeOut = 2;




  /********************************************/
  /*  Rotation Zero Offset Learn Algorithm    */
 	/********************************************/	 	

   /** KeSWRV_Deg_RZL_AbsEncdrZeroPstnDB: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Deadband Around 0 Degrees which Zero Position Detection
     * will be assumed. 
     */
    public static final double KeSWRV_Deg_RZL_AbsEncdrZeroPstnDB = 3.0;

   /** KeSWRV_Deg_RZL_AngSwpCourse: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Relative Sweep Angle for Coarse Zero Search. 
     */
    public static final double KeSWRV_Deg_RZL_AngSwpCourse = 90;

   /** KeSWRV_Deg_RZL_AngSwpFine: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Relative Sweep Angle for Fine Zero Search. 
     */
    public static final double KeSWRV_Deg_RZL_AngSwpFine = 10;

   /** KeSWRV_r_RZL_PwrSwpCourse: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Power Level for Coarse Zero Search. 
     */    
    public static final double KeSWRV_r_RZL_PwrSwpCourse = 0.20;

   /** KeSWRV_r_RZL_PwrSwpFine: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Power Level for Fine Zero Search. 
     */    
    public static final double KeSWRV_r_RZL_PwrSwpFine = 0.05;

   /** KeSWRV_t_RZL_ZeroDtctThrshIntrsv: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Zero Detection Time for the Intrusive Test. 
     */    
    public static final double KeSWRV_t_RZL_ZeroDtctThrshIntrsv = 0.250;

   /** KeSWRV_t_RZL_ZeroDtctThrshPassive: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Zero Detection Time for the Passive Learn.
     */    
    public static final double KeSWRV_t_RZL_ZeroDtctThrshPassive = 0.100;

   /** KeSWRV_t_RZL_TimeOutThrshIntrsv: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Zero Detection Safety Time-Out for the Intrusive Test. 
     */    
    public static final double KeSWRV_t_RZL_TimeOutThrshIntrsv = 3.0;



  /******************************************************/
  /*  Swerve Drive Control: Closed Loop Drive Control   */
 	/******************************************************/	 	

   /** KeSWRV_Cnt_CL_DrvErrTgtDB_Fwd: Swerve Drive System - 
     * Closed Loop Control Drive Control Error Target Deadband
     * Below which Closed-Loop will be considered Complete for
     * Longitudinal control (Forward/Rearward).
     */    
    public static final int KeSWRV_Cnt_CL_DrvErrTgtDB_Fwd = 2;

   /** KeSWRV_Cnt_CL_DrvErrTgtDB_Strafe: Swerve Drive System - 
     * Closed Loop Control Drive Control Error Target Deadband
     * Below which Closed-Loop will be considered Complete for
     * Latitudinal control or Strafing (Right/Left).
     */    
    public static final int KeSWRV_Cnt_CL_DrvErrTgtDB_Strafe = 2;

   /** KeSWRV_Deg_CL_DrvErrTgtDB_Rot: Swerve Drive System - 
     * Closed Loop Control Drive Control Error Target Deadband
     * Below which Closed-Loop will be considered Complete for
     * Rotational control (CW/CCW).
     */    
    public static final double KeSWRV_Deg_CL_DrvErrTgtDB_Rot = 1;

    
   /** KeSWRV_t_CL_DrvSyncThrshFwd: Swerve Drive System - 
     * Closed Loop Control Drive Control Sync Time for
     * Longitudinal Drive control (Forward/Reward).
     */    
    public static final double KeSWRV_t_CL_DrvSyncThrshFwd = 0.100;

    public static final double KeSWRV_t_CL_DrvSyncThrshStrafe = 0.100;

    public static final double KeSWRV_t_CL_DrvSyncThrshRot = 0.100;


    public static final double KeSWRV_k_CL_PropGx_Long = 1;
    public static final double KeSWRV_k_CL_IntglGx_Long = 0.001;
    public static final double KeSWRV_k_CL_DerivGx_Long = 0;


    public static final double KeSWRV_k_CL_PropGx_Lat = 1;
    public static final double KeSWRV_k_CL_IntglGx_Lat = 0.001;
    public static final double KeSWRV_k_CL_DerivGx_Lat = 0;

    public static final double KeSWRV_k_CL_PropGx_Rot = 1;
    public static final double KeSWRV_k_CL_IntglGx_Rot = 0.001;
    public static final double KeSWRV_k_CL_DerivGx_Rot = 0;


}
