// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Camera extends SubsystemBase {
  final static Servo Tilt = new Servo(1);
  final static Servo Pan = new Servo(2);

  final static double CAMERA_ANGLE_TOLERANCE = 2.5;

  final static double ANGLE_TILT_ZERO = 0;
  final static double ANGLE_PAN_ZERO = 0;

  public enum Targets{GOAL, BLUE_BALL, RED_BALL};

  Targets currentTarget = Targets.GOAL;

  /** Creates a new Camera. */
  public Camera() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setTilt(double position) {
    Tilt.set(position);
  }

  public double getTilt(){
    return Tilt.getPosition();
  }

  public void setPan(double position) {
    Pan.set(position);
  }

  public boolean isTargetValid(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
  }

  public double getCameraTargetXAngle() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getCameraTargetYAngle() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  // Pipeline values 0 - Goal, 1 - Blue Ball, 2 - Red Ball
  public void setCameraPipeline(Targets target){
    currentTarget = target;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(currentTarget.ordinal());
  }

  public void setNeutral() {
  }

  public boolean isTargetCenter() {
    return Math.sqrt(Math.pow(getCameraTargetXAngle(), 2) + Math.pow(getCameraTargetXAngle(), 2)) < CAMERA_ANGLE_TOLERANCE;
  }

  public void moveCameraToCenter() {
    
  }
}
