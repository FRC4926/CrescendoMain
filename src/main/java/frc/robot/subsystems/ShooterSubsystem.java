// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
 public CANSparkMax frontLeft = new CANSparkMax(6, MotorType.kBrushless);
 public CANSparkMax frontRight = new CANSparkMax(5, MotorType.kBrushless);


  /** Creates a new DriveSubsystem. */
  public ShooterSubsystem() {

    frontRight.restoreFactoryDefaults();
    frontLeft.restoreFactoryDefaults();
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void drive(double moveLeft, double moveRight){
      frontLeft.set(moveLeft);
      frontRight.set(moveRight);

  }
  public double getShooterAngle(){
    return (Math.atan((Constants.FieldConstant.speakerTagHeight-Constants.RobotParameters.cameraHeight)/RobotContainer.Subsystems.m_limelightSubsystem.calcHorizontalDistance()));
  }
}
