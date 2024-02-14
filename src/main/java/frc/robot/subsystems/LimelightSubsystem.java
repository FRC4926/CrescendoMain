// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.util.GalacPIDController;

public class LimelightSubsystem extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tid = table.getEntry("tid");
  NetworkTableEntry tz = table.getEntry("tz");
  int pipelineNum = 0;
  // read values periodically
  double x = tx.getDouble(0.0);
  public double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double id = tid.getDouble(0.0);
  double z;
 double turnEffort;
  double driveEffort;
  GalacPIDController alignPidController = new GalacPIDController(0.015, 0, 0, 0.1, () -> getX(), 0, 2);
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
  }

  public void updateLimelight() {
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tz = table.getEntry("tz");
    tid = table.getEntry("tid");
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    z = tz.getDouble(0.0);
    area = ta.getDouble(0.0);
    id = tid.getDouble(0.0);
  }

  public void setPipeline(int pipeline) {
    pipelineNum = pipeline;
    NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    pipelineEntry.setNumber(pipeline);
  }

  public void align(){
    updateLimelight();
    alignPidController.setSetpoint(getOffSet());
    turnEffort = alignPidController.getEffort();
    RobotContainer.Subsystems.m_driveSubsystem.drive(0, turnEffort);
  }
  public void alignForSourceIntake(){
    updateLimelight();
    alignPidController.setSetpoint(getOffSet());
    turnEffort = alignPidController.getEffort();
    RobotContainer.Subsystems.m_driveSubsystem.drive(0, turnEffort);
  }
  public double getX() {
    return x;
  }
  public double getOffSet(){
return Math.abs(Math.atan(Constants.Robot.limeLightOffSet/calcVerticalDistance()));
  }
    public double getSourceOffSet(){
return Math.abs(Math.atan(Constants.Robot.limeLightOffSet/calcVerticalDistance()));
  }

  public double getY() {
    return y;
  }
  public boolean isFinsished(){
    //4.5 is calculated
    return Math.abs(alignPidController.getSetpoint()-getX())<4.5 && Subsystems.m_driveSubsystem.frontLeftMotor.getEncoder().getVelocity()<.1;
  }

  public double getID() {
    return id;
  }

  public double getArea() {
    return area;
  }

  public double calcVerticalDistance() {

    double pitch = Math
        .toRadians(Constants.Robot.cameraAngle +
            RobotContainer.Subsystems.m_driveSubsystem.getGyroPitch());
    // target height - camera height
    double dh = Constants.Field.speakerTagHeight - Constants.Robot.cameraHeight;
    return dh / Math.tan(Math.toRadians(y + pitch));
  }
  public double calcVerticalDistanceSource() {

    double pitch = Math
        .toRadians(Constants.Robot.cameraAngle +
            RobotContainer.Subsystems.m_driveSubsystem.getGyroPitch());
    // target height - camera height
    double dh = Constants.Field.sourceTagHeight - Constants.Robot.cameraHeight;
    return dh / Math.tan(Math.toRadians(y + pitch));
  }

  @Override
  public void periodic() {

  }
}
