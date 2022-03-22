// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.calibrations.K_INTK;
import frc.robot.commandgroups.CG_DrvBack;
import frc.robot.commandgroups.CG_ShootLowDrvBack;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public  final RFSLIB prfsLIB = new RFSLIB();
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
  // private final SwerveDriveSubsystem swerveSubsystem = new SwerveDriveSubsystem();
  private final CompressorSubsystem compressorSubsystem = new CompressorSubsystem();
  private final SwerveDrivetrain swerveSubsystem = new SwerveDrivetrain();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();
  private final Camera cameraSubsystem = new Camera();
  private XboxController driverStick = new XboxController(Constants.DRVR_CNTRLR);
  private XboxController auxStick = new XboxController(Constants.AUX_CNTRLR);
  
  private Command m_autoCommandSelected;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  // Configure Autonomous Selections Available
     m_chooser.setDefaultOption("Default Auto", new CC_IntakeArmsRaise(intakeSubsystem));
//     m_chooser.addOption("Test Drive", new Auto_Drive_Deadrecken(swerveSubsystem, 0, 0.5, 0, 1));
//     m_chooser.addOption("Test Drive Encoder Reverse", new SwerveLongDistEncdr(swerveSubsystem, 36, false));  
//     m_chooser.addOption("Test Drive Encoder Right", new SwerveLatDistEncdr(swerveSubsystem, 36, true));
     m_chooser.addOption("Do Nothing", new CC_IntakeArmsRaise(intakeSubsystem));
     m_chooser.addOption("Just Shoot", new CA_Shoot(shooterSubsystem, intakeSubsystem, cameraSubsystem, false));
     m_chooser.addOption("Drive Back", new CG_DrvBack(swerveSubsystem, intakeSubsystem));
     m_chooser.addOption("Shoot and Drive Back", new CG_ShootLowDrvBack(swerveSubsystem, shooterSubsystem, intakeSubsystem, cameraSubsystem));
     SmartDashboard.putData("Auto choices: ", m_chooser);
     

    // Configure the button bindings
    configureButtonBindings();
    // Configure Default Commands
    setDefaultCommands();

  }


  /**
   * Use this method to schedule any subsystem initialization tasks required to be run
   * at the initialization of the Autonomous Period.
   */
  public void autonomousInit() {
    liftSubsystem.init_periodic();
    intakeSubsystem.init_periodic();
    shooterSubsystem.init_periodic();
  }


  /**
   * Use this method to schedule any subsystem initialization tasks required to be run
   * at the initialization of the Tele-Op Period.
   */
  public void teleopInit() {
    liftSubsystem.init_periodic();
    intakeSubsystem.init_periodic();
    shooterSubsystem.init_periodic();
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* BEGIN DRIVER STICK BUTTON ASSIGNMENTS */
    final JoystickButton driverButton_Start = new JoystickButton (driverStick, Constants.BUTTON_START);

    driverButton_Start.whenPressed(new CT_LiftRobot(liftSubsystem, auxStick));


    /* BEGIN AUXILLARY STICK BUTTON ASSIGNMENTS */

    final Button rightTriggerButton = new Button(() -> (auxStick.getRightTriggerAxis() >= K_INTK.KeINTK_r_IntakeMtrTriggerLvlEnbl));
    final JoystickButton auxButton_A = new JoystickButton(auxStick, Constants.BUTTON_A);
    final JoystickButton auxButton_B = new JoystickButton(auxStick, Constants.BUTTON_B);
    final JoystickButton auxButton_X = new JoystickButton(auxStick, Constants.BUTTON_X);
    final JoystickButton auxButton_Y = new JoystickButton(auxStick, Constants.BUTTON_Y);
    final JoystickButton auxButton_BumpLT = new JoystickButton(auxStick, Constants.BUMPER_LEFT);
    final JoystickButton auxButton_BumpRT = new JoystickButton(auxStick, Constants.BUMPER_RIGHT);

    rightTriggerButton.whenPressed(new CT_IntakeBalls(intakeSubsystem, auxStick));
    auxButton_A.whileHeld(new CT_Shoot(shooterSubsystem, intakeSubsystem, cameraSubsystem, false));  // Shoot Low Target
    auxButton_B.whileHeld(new CT_Shoot(shooterSubsystem, intakeSubsystem, cameraSubsystem, true));   // Shoot High Target
    auxButton_X.whenPressed(new CC_IntakeArmsLower(intakeSubsystem));
    auxButton_Y.whenPressed(new CC_IntakeArmsRaise(intakeSubsystem));
    auxButton_BumpLT.whenPressed(new CC_ShooterServoCmd(shooterSubsystem,-1));
    auxButton_BumpRT.whenPressed(new CC_ShooterServoCmd(shooterSubsystem,1));

  }


  private void setDefaultCommands() {
    // CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, new ManualDrive(swerveSubsystem, driverStick));

                                                   // Subsystem, Control Joystick, fieldCentric, openLoop
    swerveSubsystem.setDefaultCommand(new CT_SwerveDrive(swerveSubsystem, driverStick, false, true));
    cameraSubsystem.setDefaultCommand(new CC_CameraTrackTarget(cameraSubsystem));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_autoCommandSelected = m_chooser.getSelected(); 
    System.out.println("Auto selected: " + m_autoCommandSelected);
    return(m_autoCommandSelected);
  }

    
}