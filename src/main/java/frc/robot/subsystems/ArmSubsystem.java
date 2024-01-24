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
  public CANSparkMax armMotor1 = new CANSparkMax(Constants.CAN_IDS.SHOULDER1, MotorType.kBrushless);
  public CANSparkMax armMotor2 = new CANSparkMax(Constants.CAN_IDS.SHOULDER1, MotorType.kBrushless);
  public double angle = armMotor1.getEncoder().getPosition() + Constants.RobotParameters.wristAngle;
  public PIDController armController = new PIDController(p, i, d);
  private LookUpTableShooterAngles angles = new LookUpTableShooterAngles();

  public ArmSubsystem() {
    armMotor1.restoreFactoryDefaults();
    armMotor1.getEncoder().setPosition(Constants.RobotParameters.initialShoulderAngle / 360);
    armMotor2.restoreFactoryDefaults();
    armMotor2.getEncoder().setPosition(Constants.RobotParameters.initialShoulderAngle / 360);
  }

  public double getArmDesiredAngle() {
    return (angles.lookUpAngle(RobotContainer.Subsystems.m_limelightSubsystem.calcHorizontalDistance()));
  }

  public double getArmActualAngle() {
    return (armMotor1.getEncoder().getPosition() + armMotor1.getEncoder().getPosition()) / (4 * Math.PI)
        * Constants.RobotParameters.shoulderGearRation + Constants.RobotParameters.wristAngle;
  }

  public void adjustArmShooterAngle() {
    armController.setSetpoint(getArmDesiredAngle());
    armMotor1.set(armController.calculate(getArmActualAngle()));
  }

  public void goToHome() {
    armController.setSetpoint(Constants.RobotParameters.initialShoulderAngle + Constants.RobotParameters.wristAngle);
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
