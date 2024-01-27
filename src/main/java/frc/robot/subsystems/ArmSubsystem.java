// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.LookUpTableShooterAngles;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  double p = 0.01;
  double i = 0;
  double d = 0;
  public CANSparkMax armMotor1 = new CANSparkMax(Constants.CAN_IDS.ARM_LEFT, MotorType.kBrushless);
  public CANSparkMax armMotor2 = new CANSparkMax(Constants.CAN_IDS.ARM_RIGHT, MotorType.kBrushless);
  public double angle = armMotor1.getEncoder().getPosition() + Constants.Robot.wristAngle;
  public PIDController armController = new PIDController(p, i, d);
  private LookUpTableShooterAngles angles = new LookUpTableShooterAngles();

  public ArmSubsystem() {
    armMotor1.restoreFactoryDefaults();
    armMotor1.getEncoder().setPosition(Constants.Robot.initialShoulderAngle / 360);
    armMotor2.restoreFactoryDefaults();
    armMotor2.getEncoder().setPosition(Constants.Robot.initialShoulderAngle / 360);
  }

  public double getArmDesiredAngle() {
        if(RobotContainer.Subsystems.m_limelightSubsystem.id== 7 || RobotContainer.Subsystems.m_limelightSubsystem.id== 7 )
    return (angles.lookUpAngle(RobotContainer.Subsystems.m_limelightSubsystem.calcHorizontalDistance()));
    else
    return Constants.Robot.ampAngle;
  }

  public double getArmActualAngle() {
    return (armMotor1.getEncoder().getPosition() + armMotor1.getEncoder().getPosition()) / (4 * Math.PI)
        * Constants.Robot.shoulderGearRation + Constants.Robot.wristAngle;
  }

  public void adjustArmShooterAngle() {
    armController.setSetpoint(getArmDesiredAngle());
    armMotor1.set(armController.calculate(getArmActualAngle()));
  }

  public void goToHome() {
    armController.setSetpoint(Constants.Robot.initialShoulderAngle + 
    Constants.Robot.wristAngle);
    armMotor1.set(armController.calculate(getArmActualAngle()));
  }

  public void manualControl(double input) {
    armMotor1.set(input);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
