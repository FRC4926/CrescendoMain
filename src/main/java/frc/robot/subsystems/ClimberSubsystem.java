// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  public CANSparkMax climber = new CANSparkMax(Constants.CAN_IDS.CLIMBER, MotorType.kBrushless);
  public boolean climbed = false;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climber.setIdleMode(IdleMode.kBrake);
    climber.setSmartCurrentLimit(40);
    //climber.setInverted(true);
  }

  public boolean getClimbed(){
    return climbed;
  }

  public void resetEncoders() {
    climber.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb(double effort) {
    SmartDashboard.putNumber("Climber Encoder", climber.getEncoder().getPosition());
    if (climber.getEncoder().getPosition() > 20 && effort > 0) {
      climber.set(0);
    } else if (climber.getEncoder().getPosition() < Constants.Robot.climberMaxEncoderPosition && effort < 0) {
      climbed = true;
      climber.set(0);
    } else {
      climber.set(effort);
    }
  }
}
