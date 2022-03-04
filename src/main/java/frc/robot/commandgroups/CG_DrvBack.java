// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.commands.TimeDly;
import frc.robot.commands.IntakeArmsRaise;
import frc.robot.commands.SwerveLongDistEncdr;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_DrvBack extends SequentialCommandGroup {
  /** Creates a new CG_DrvBack. */
  public CG_DrvBack(SwerveDrivetrain swerveDrivetrain,
                             IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      (new SwerveLongDistEncdr(swerveDrivetrain, 90, false)),
      (new TimeDly(0.5)),
      (new IntakeArmsRaise(intakeSubsystem))
    );
    System.out.println("CG_DrvBack3 Autonomous Invoked.");
  }
}
