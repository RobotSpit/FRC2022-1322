// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.commands.TimeDly;
import frc.robot.commands.AutoShootLo;
import frc.robot.commands.IntakeArmsRaise;
import frc.robot.commands.SwerveLatDistEncdr;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_ShootLowDrvBack extends SequentialCommandGroup {
  /** Creates a new CG_ShootLowDrvBack. */
  public CG_ShootLowDrvBack(SwerveDrivetrain swerveDrivetrain,
                             ShooterSubsystem shooterSubsystem,
                             IntakeSubsystem intakeSubsystem,
                             Camera cameraSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      (new AutoShootLo(shooterSubsystem, intakeSubsystem, cameraSubsystem)),
      (new TimeDly(0.5)),
      (new SwerveLatDistEncdr(swerveDrivetrain, 90, false)),
      (new TimeDly(0.5)),
      (new IntakeArmsRaise(intakeSubsystem))
    );
    System.out.println("CG_ShootLowAndBackUp Autonomous Invoked.");
  }
}
