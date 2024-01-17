// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  public CANSparkMax frontLeftMotor = new CANSparkMax(Constants.CAN_IDS.FRONT_LEFT, MotorType.kBrushless);
  public CANSparkMax backLeftMotor = new CANSparkMax(Constants.CAN_IDS.BACK_LEFT, MotorType.kBrushless);
  public CANSparkMax frontRightMotor = new CANSparkMax(Constants.CAN_IDS.FRONT_RIGHT, MotorType.kBrushless);
  public CANSparkMax backRightMotor = new CANSparkMax(Constants.CAN_IDS.BACK_RIGHT, MotorType.kBrushless);
  public RelativeEncoder leftEncoder = frontLeftMotor.getEncoder();
  public RelativeEncoder rightEncoder= frontRightMotor.getEncoder();
  DifferentialDrive differentialDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontLeftMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    rightEncoder.setPositionConversionFactor(Constants.RobotParameters.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(Constants.RobotParameters.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(Constants.RobotParameters.kLinearDistanceConversionFactor / 60);
    leftEncoder.setVelocityConversionFactor(Constants.RobotParameters.kLinearDistanceConversionFactor / 60);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);
    //navX.reset();
    resetEncoders();
  }
  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  public void tankDriveVolts(Measure<Voltage> vots) {
    frontLeftMotor.setVoltage(vots.magnitude());
    frontRightMotor.setVoltage(vots.magnitude());
    differentialDrive.feed();
  }
  public void logMotors(SysIdRoutine sys){

  }
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                frontLeftMotor.setVoltage(volts.in(Volts));
                frontRightMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(leftEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(leftEncoder.getVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(rightEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(rightEncoder.getVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }
public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> differentialDrive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
