// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.calibrations.K_INTK;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final SwerveDrivetrain swerveSubsystem = new SwerveDrivetrain();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();
  private XboxController driverStick = new XboxController(Constants.DRVR_CNTRLR);
  private XboxController auxStick = new XboxController(Constants.AUX_CNTRLR);
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    // Configure Default Commands
    setDefaultCommands();

  }



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

    /* BEGIN AUXILLARY STICK BUTTON ASSIGNMENTS */

    final Button rightTriggerButton = new Button(() -> (auxStick.getRightTriggerAxis() >= K_INTK.KeINTK_r_IntakeMtrTriggerLvlEnbl));
    final Button liftTrigger = new Button(() -> (auxStick.getStartButtonPressed() && auxStick.getBackButtonPressed()));
    final JoystickButton driverButton_Start = new JoystickButton (auxStick, Constants.BUTTON_START);
    final JoystickButton auxButton_A = new JoystickButton(auxStick, Constants.BUTTON_A);
    final JoystickButton auxButton_X = new JoystickButton(auxStick, Constants.BUTTON_X);
    final JoystickButton auxButton_Y = new JoystickButton(auxStick, Constants.BUTTON_Y);
    final JoystickButton auxButton_BumpLT = new JoystickButton(auxStick, Constants.BUMPER_LEFT);
    // final JoystickButton auxDPAD = new JoystickButton(auxStick, Constants.DPAD);

    rightTriggerButton.whenPressed(new ManualIntake(intakeSubsystem, auxStick));
    auxButton_A.whileHeld(new ManualShoot(shooterSubsystem, intakeSubsystem, auxButton_A));
    auxButton_X.whenPressed(new IntakeArmsLower(intakeSubsystem));
    auxButton_Y.whenPressed(new IntakeArmsRaise(intakeSubsystem));
    // auxButton_BumpLT.whenPressed(new SwerveZeroPointLearn(swerveSubsystem));
    driverButton_Start.whenPressed(new ManualLift(liftSubsystem, auxStick));
    //liftTrigger.whenPressed(new ManualLift(liftSubsystem, auxStick));

  }


  private void setDefaultCommands() {
    // CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, new ManualDrive(swerveSubsystem, driverStick));

                                                   // Subsystem, Control Joystick, fieldCentric, openLoop
    swerveSubsystem.setDefaultCommand(new SwerveTeleop(swerveSubsystem, driverStick, false, true));
    // CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem, new ManualIntake(intakeSubsystem, auxStick));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }


    
}