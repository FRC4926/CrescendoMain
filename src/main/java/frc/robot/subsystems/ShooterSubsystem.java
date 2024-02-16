// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
 public CANSparkMax bottom = new CANSparkMax(Constants.CAN_IDS.SHOOTER_BOTTOM, MotorType.kBrushless);
 public CANSparkMax top = new CANSparkMax(Constants.CAN_IDS.SHOOTER_TOP, MotorType.kBrushless);
double desiredRPM = 4900;
double rpmTolerance = 100;
SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(-0.73956, 0, 0);


  /** Creates a new DriveSubsystem. */
  public ShooterSubsystem() {

    top.restoreFactoryDefaults();
    bottom.restoreFactoryDefaults();
    top.setSmartCurrentLimit(50);
    bottom.setSmartCurrentLimit(50);
    top.setIdleMode(IdleMode.kCoast);
    bottom.setIdleMode(IdleMode.kCoast);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  PIDController rpmController = new PIDController(0.0012, 0.00015, 0);
  public void shoot(){
      bottom.set(shooterFeedForward.calculate(desiredRPM));
      top.set(shooterFeedForward.calculate(desiredRPM));
  }
  public void shootAmp(){
    bottom.set(-.3);
    top.set(-.3);
  }
  public boolean isFinished(){
    return top.getEncoder().getVelocity()>desiredRPM-rpmTolerance;
  }
  public void idle(){
    top.set(-.1);
    bottom.set(-.1);
  }
  public void stop(){
    bottom.set(0);
    top.set(0);
  }
 

}