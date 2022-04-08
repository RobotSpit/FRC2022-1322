// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.commands.CC_TimeDly;
import frc.robot.commands.CA_IntakeBall;
import frc.robot.commands.CA_Shoot;
import frc.robot.commands.CA_SwerveLongDistEncdr;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.*;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_ShootHighDrvBackParallel extends SequentialCommandGroup {
  /** Creates a new CG_ShootLowDrvBackParallel. */
  public CG_ShootHighDrvBackParallel(SwerveDrivetrain swerveDrivetrain,
                                     ShooterSubsystem shooterSubsystem,
                                     IntakeSubsystem intakeSubsystem,
                                     Camera cameraSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      (new CA_Shoot(shooterSubsystem, intakeSubsystem, cameraSubsystem, slctGoal.HiGoalClose)),
      (new CC_TimeDly(0.5)),
      (new CG_DriveAndGrabBall(swerveDrivetrain, intakeSubsystem)),
      (new CA_SwerveLongDistEncdr(swerveDrivetrain, 0.3, 1, true)),
      (new CA_IntakeBall(intakeSubsystem)),
      (new CC_TimeDly(0.25)),
      (new CA_Shoot(shooterSubsystem, intakeSubsystem, cameraSubsystem, slctGoal.HiGoalMid))
    );
    System.out.println("CG_ShootHighDrvBack Autonomous Invoked.");
  }
}
