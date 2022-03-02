package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.utils.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveDrivetrain extends SubsystemBase {
    
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] swerveModules;
    private AHRS gyro;
    private Field2d field;

    public SwerveDrivetrain() {
        this.gyro = new AHRS(Constants.SwerveDrivetrain.GYRO_ID);
        this.gyro.calibrate();
        this.zeroGyro();

        this.swerveOdometry = new SwerveDriveOdometry(Constants.SwerveDrivetrain.SWERVE_KINEMATICS, this.getYaw());

        this.swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveDrivetrain.Mod0.constants),
            new SwerveModule(1, Constants.SwerveDrivetrain.Mod1.constants),
            new SwerveModule(2, Constants.SwerveDrivetrain.Mod2.constants),
            new SwerveModule(3, Constants.SwerveDrivetrain.Mod3.constants)
        };

        this.field = new Field2d();

        dashboard();
    }

    public void print() {
        
        this.swerveModules[0].getCanCoder();
        this.swerveModules[1].getCanCoder();
        this.swerveModules[2].getCanCoder();
        this.swerveModules[3].getCanCoder();
        
        SmartDashboard.putNumber("Fixed Gyro: ", this.getYaw().getDegrees());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveDrivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    this.getYaw()
                )
            :
                new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation
                )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrivetrain.MAX_SPEED);

        for (SwerveModule mod : this.swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Gyro */
    public void zeroGyro() {
        this.gyro.reset();
    }

    private double optimizeGyro (double degrees) {
        // 0 < degrees < 360
        if ((degrees > 0.0) && (degrees < 360.0)) {
            return degrees;
        } else {
            int m = (int) Math.floor( degrees / 360.0 );
            double optimizedDegrees = degrees - (m * 360.0);
            return Math.abs(optimizedDegrees);
        }
    }

    public Rotation2d getYaw() {
        double pitch = this.gyro.getPitch();
        double roll = this.gyro.getRoll();
        double rawYaw = this.gyro.getYaw();
        SmartDashboard.putNumber("Roll Gyro: ", roll);
        SmartDashboard.putNumber("Pitch Gyro: ", pitch);
        SmartDashboard.putNumber("Yaw Gyro: ", rawYaw);
        double yaw = optimizeGyro(pitch);
        return Constants.SwerveDrivetrain.INVERT_GYRO ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    }

    public double getGyroAngleDegrees() {
        return this.getYaw().getDegrees();
    }

    public double getGyroAngleRadians() {
        return this.getYaw().getRadians();
    }

    public void resetToAbsolute() {
        this.swerveModules[0].resetToAbsolute();
        this.swerveModules[1].resetToAbsolute();
        this.swerveModules[2].resetToAbsolute();
        this.swerveModules[3].resetToAbsolute();
    }

    /* Odometry */
    public Pose2d getPose() {
        return this.swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        this.swerveOdometry.resetPosition(pose, pose.getRotation());
    }

    public void resetOdometry(Pose2d pose) {
        this.swerveOdometry.resetPosition(pose, this.getYaw());
    }

    // public boolean runUntilZero() {
    //     double encoder0 = this.swerveModules[0].getCanCoder().getDegrees() - this.swerveModules[0].getAngleOffset();
    //     double encoder1 = this.swerveModules[1].getCanCoder().getDegrees() - this.swerveModules[1].getAngleOffset();
    //     double encoder2 = this.swerveModules[2].getCanCoder().getDegrees() - this.swerveModules[2].getAngleOffset();
    //     double encoder3 = this.swerveModules[3].getCanCoder().getDegrees() - this.swerveModules[3].getAngleOffset();
    //     boolean allAtZero = true;
    //     if(encoder0 > 3) {
    //         this.swerveModules[0].runAngleByPercent(.2);
    //         allAtZero = false;
    //     } else {
    //         this.swerveModules[0].runAngleByPercent(0);
    //     }
    //     if(encoder1 > 3) {
    //         this.swerveModules[1].runAngleByPercent(.2);
    //         allAtZero = false;
    //     } else {
    //         this.swerveModules[1].runAngleByPercent(0);
    //     }
    //     if(encoder2 > 3) {
    //         this.swerveModules[2].runAngleByPercent(.2);
    //         allAtZero = false;
    //     } else {
    //         this.swerveModules[2].runAngleByPercent(0);
    //     }
    //     if(encoder3 > 3) {
    //         this.swerveModules[3].runAngleByPercent(.2);
    //         allAtZero = false;
    //     } else {
    //         this.swerveModules[3].runAngleByPercent(0);
    //     }
    //     return allAtZero;
    // }

    /* Module States */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : this.swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveDrivetrain.MAX_SPEED);
        for (SwerveModule mod : this.swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void dashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.add(this);
        tab.addNumber("Gyro Angle ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGyro);
        tab.addNumber("Gyro Angle (GRAPH) ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGraph);
        SmartDashboard.putData(this.field);
        // SmartDashboard.putData("ANGLE PID", data);
        // SmartDashboard.putData("DRIVE PID", data);
    }

    @Override
    public void periodic() {
        this.print();
        this.swerveOdometry.update(this.getYaw(), this.getStates());
        this.field.setRobotPose(this.swerveOdometry.getPoseMeters());
    }
}
