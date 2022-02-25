// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Enum for Solenoid control
    public enum SolenoidPosition {
        UP, DOWN, OFF
    }

   /* MOTOR CAN ADDRESS ASSIGNMENTS */
    // NOTE: These are tentative and are subject to change
    // Drive Motors (FALCON 500)
    public static final int DRV_MTR_FT_LT           = 1;  //  Front Left
    public static final int DRV_MTR_RR_LT           = 2;  //  Rear  Left
    public static final int DRV_MTR_FT_RT           = 3;  //  Front Right
    public static final int DRV_MTR_RR_RT           = 4;  //  Rear Right
    // Swerve Steer Motors (FALCON 500)
    public static final int SWRV_MTR_FT_LT          = 5;  //  Front Left
    public static final int SWRV_MTR_RR_LT          = 6;  //  Rear  Left
    public static final int SWRV_MTR_FT_RT          = 7;  //  Front Right
    public static final int SWRV_MTR_RR_RT          = 8;  //  Rear Right

    
    // Shooter Motors (SPARK MAX)
    public static final int SHOOTER1                = 9;
    public static final int SHOOTER2                = 10;
    // Robot Lift (SPARK MAX)
    public static final int ROBOT_LIFT              = 11;
    // Shooter Tilt (CAN TALON SRX)
    public static final int SHOOTER_AIM_TILT        = 12;
    // Shooter PAN (CAN TALON SRX)
    public static final int SHOOTER_AIM_PAN         = 13;
    // Shooter Intake (CAN TALON SRX)
    public static final int SHOOTER_BALL_INTAKE     = 14;
    // Shooter Intake Lift (CAN TALON SRX)
    public static final int SHOOTER_INTAKE_LIFT     = 15;
    // Shooter Advance (CAN TALON SRX)
    public static final int SHOOTER_BALL_ADVANCE    = 16;


    /* Analog Inputs */
    // Swerve Steer Motors Position
    public static final int SWRV_ANG_FT_LT          = 3;  //  Front Left
    public static final int SWRV_ANG_RR_LT          = 2;  //  Rear  Left
    public static final int SWRV_ANG_FT_RT          = 0;  //  Front Right
    public static final int SWRV_ANG_RR_RT          = 1;  //  Rear Right



    /* Digital Inputs */
    public static final int BALL_SENSE_INPUT    = 1;
    public static final int BALL_SENSE_OUTPUT   = 2;    
    public static final int SWRV_ZERO_FT_RT     = 4;
    public static final int SWRV_ZERO_FT_LT     = 6;
    public static final int SWRV_ZERO_RR_LT     = 5;
    public static final int SWRV_ZERO_RR_RT     = 3;

   /* Pneumatic Actuators */
    public static final int BALL_INTAKE_BLOCK   = 0;
    public static final int BALL_INTAKE_OPEN    = 1;
    public static final int HANG_HOOK_UP        = 2;
    public static final int HANG_HOOK_RETRACT   = 3;



    /* X-BOX CONTROLLER MAPPING */
    // Controller Assignments
    public static final int DRVR_CNTRLR       =  0;
    public static final int AUX_CNTRLR        =  1;
    // Button Assignments
    public static final int BUTTON_A          =  1;
    public static final int BUTTON_B          =  2;
    public static final int BUTTON_X          =  3;
    public static final int BUTTON_Y          =  4;
    public static final int BUMPER_LEFT       =  5;
    public static final int BUMPER_RIGHT      =  6;
    public static final int BUTTON_BACK       =  7;  // LEFT(SELECT)
    public static final int BUTTON_START      =  8;  // RIGHT
    public static final int STICK_LEFT_PRESS  =  9;  // JOYSTICK PRESS
    public static final int STICK_RIGHT_PRESS = 10;  // JOYSTICK PRESS
    // Analog Assignments
    public static final int STICK_LEFT_XAXIS  = 1;
    public static final int STICK_LEFT_YAXIS  = 2;
    public static final int TRIGGERS          = 3;
    public static final int STICK_RIGHT_XAXIS = 4;
    public static final int STICK_RIGHT_YAXIS = 5;
    public static final int DPAD              = 6;


}
