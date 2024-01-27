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
 public CANSparkMax bottom = new CANSparkMax(Constants.CAN_IDS.SHOOTER_BOTTOM, MotorType.kBrushless);
 public CANSparkMax top = new CANSparkMax(Constants.CAN_IDS.SHOOTER_TOP, MotorType.kBrushless);


  /** Creates a new DriveSubsystem. */
  public ShooterSubsystem() {

    top.restoreFactoryDefaults();
    bottom.restoreFactoryDefaults();
    top.setSmartCurrentLimit(50);
    bottom.setSmartCurrentLimit(50);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shoot(double moveLeft, double moveRight){
      bottom.set(moveLeft);
      top.set(moveRight);

  }
  public double getCurrentAngle(){
    return 0;
  }

}
