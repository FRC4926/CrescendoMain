// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
 public CANSparkMax bottomMotor = new CANSparkMax(Constants.CAN_IDS.SHOOTER_BOTTOM, MotorType.kBrushless);
 public CANSparkMax topMotor = new CANSparkMax(Constants.CAN_IDS.SHOOTER_TOP, MotorType.kBrushless);
 public PIDController bottomShooterController = new PIDController(Constants.Robot.shooterP, 0, 0);
 public PIDController topShooterController = new PIDController(Constants.Robot.shooterP, 0, 0);
 public RelativeEncoder bottom = bottomMotor.getEncoder();
 public RelativeEncoder top = topMotor.getEncoder();
  /** Creates a new DriveSubsystem. */
  public ShooterSubsystem() {
    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();
    topMotor.setSmartCurrentLimit(50);
    bottomMotor.setSmartCurrentLimit(50);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public boolean rev(double topRPM, double bottomRPM){
      bottomMotor.set(bottomShooterController.calculate(bottom.getVelocity(), bottomRPM));
      topMotor.set(topShooterController.calculate(top.getVelocity(),topRPM));
      if(Math.abs(bottom.getVelocity()-bottomRPM)<Constants.Robot.shooterMargins && Math.abs(top.getVelocity()-topRPM)<Constants.Robot.shooterMargins){
        return true;
      }else{
        return false;
      }
  }
  // public double getCurrentAngle(){
  //   return 0;
  // }

}
