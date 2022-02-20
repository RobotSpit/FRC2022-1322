// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public  final pRFSLIB prfsLIB = new pRFSLIB();
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
  private final SwerveDriveSubsystem swerveSubsystem = new SwerveDriveSubsystem();
  private final BallSubsystem ballSubsystem = new BallSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();
  private XboxController driverStick;
  private XboxController auxStick;
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  private int debugInstrDisplayUpdCounter;
  public boolean debugInstrDisplayUpdNow;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    debugInstrDisplayUpdCounter = (int)0;
    debugInstrDisplayUpdNow = true;



    // Configure the button bindings
    configureButtonBindings();
    // Configure Default Commands
    setDefaultCommands();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* BEGIN DRIVER STICK BUTTON ASSIGNMENTS */
    driverStick = new XboxController(Constants.DRVR_CNTRLR);

    /* BEGIN AUXILLARY STICK BUTTON ASSIGNMENTS */
    auxStick = new XboxController(Constants.AUX_CNTRLR); 

  }


  private void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, new SWRV_DrvManual(swerveSubsystem, driverStick));
  }



  public SwerveDriveSubsystem getSwerveDriveSubsystem()
    {
    return(swerveSubsystem);
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


  /**
   * Method: updateInstrDisplayUpd{}
   * Use this to Service the Instrumenation Display Update Counter and Flag.
   * Used to Trigger the Debug Insturmentation Display Flag to update data
   * display every 10 loops so that it will not bog down processing.
   * 
   * @return none
   */
  private void updateInstrDisplayUpd() {
    if (debugInstrDisplayUpdCounter == (int)0) {
      debugInstrDisplayUpdNow = true;  
    } else {
      debugInstrDisplayUpdNow = false;  
    }

    debugInstrDisplayUpdCounter++;
    if (debugInstrDisplayUpdCounter >= (int)11) {
      debugInstrDisplayUpdCounter = (int)0;
    }

  }

    
}