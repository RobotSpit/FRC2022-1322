package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveTeleop extends CommandBase {

    private static final double DEADBAND = 0.1;

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveDrivetrain s_Swerve;
    private XboxController controller;

    public SwerveTeleop (SwerveDrivetrain s_Swerve, XboxController controller, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        // double yAxis = -controller.getLeftY();
        // double xAxis = -controller.getLeftX();
        // double rAxis = -controller.getRightX();

        double yAxis = -controller.getRawAxis(1);
        double xAxis = -controller.getRawAxis(0);
        double rAxis = -controller.getRawAxis(4);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < DEADBAND) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < DEADBAND) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < DEADBAND) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveDrivetrain.MAX_SPEED);
        rotation = rAxis * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
