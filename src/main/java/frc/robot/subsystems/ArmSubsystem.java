// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Controllers;
import frc.robot.util.LookUpTableShooterAngles;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  double p = 9;
  double i = 0;
  double d = 0;
  double ffVelocity = 1;
  // how often we want to update our trapezoidal profiling
  double dT = 0.02;
  public double targetAngle = 0;
  //ArmFeedforward ff = new ArmFeedforward(0, 1.32, 1.95, 0.07);
  public ArmFeedforward ff = new ArmFeedforward(0, 1.50, 0, 0);
  public CANSparkMax armMotor = new CANSparkMax(Constants.CAN_IDS.ARM, MotorType.kBrushless);

  public double angle = armMotor.getEncoder().getPosition();
  public PIDController armController = new PIDController(p, i, d);
  private LookUpTableShooterAngles angles = new LookUpTableShooterAngles();
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 3));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  public ArmSubsystem() {
    armMotor.restoreFactoryDefaults();
    // armMotor2.restoreFactoryDefaults();

    armMotor.setIdleMode(IdleMode.kBrake);
    // armMotor2.setIdleMode(IdleMode.kBrake);

    armMotor.setInverted(true);

    armMotor.getEncoder().setPositionConversionFactor((2*Math.PI)*(0.01));
    // armMotor2.getEncoder().setPositionConversionFactor(Constants.Robot.shoulderGearRatio*360);



    armMotor.setSmartCurrentLimit(60);
    // armMotor2.getEncoder().setPosition(Constants.Robot.initialShoulderAngle);

  }

  public double getArmDesiredAngle() {
    if (RobotContainer.Subsystems.m_limelightSubsystem.id == 7
        || RobotContainer.Subsystems.m_limelightSubsystem.id == 4)
      return (angles.lookUpAngle(RobotContainer.Subsystems.m_limelightSubsystem.calcVerticalDistance()));
    else
      return -1;
  }

  public double getArmActualAngle() {
    // return (armMotor.getEncoder().getPosition() + armMotor2.getEncoder().getPosition()) / (2);
    return armMotor.getEncoder().getPosition();
  }

  public void adjustArmShooterAngle() {
    if (getArmDesiredAngle() == -1) {
      RobotContainer.Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, .5);
    } else {
      armController.setSetpoint(getArmDesiredAngle());
      armMotor
          .setVoltage(armController.calculate(getArmActualAngle()) + ff.calculate(getArmDesiredAngle(), ffVelocity));
      // armMotor2
      //     .setVoltage(armController.calculate(getArmActualAngle()) + ff.calculate(getArmDesiredAngle(), ffVelocity));
      RobotContainer.Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  public void trapezoidAdjustArmShooterAngle() {
    if (getArmDesiredAngle() == -1) {
      RobotContainer.Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, .5);
    } else {
      m_goal = new TrapezoidProfile.State(getArmDesiredAngle(), 0);
      m_setpoint = m_profile.calculate(dT, m_setpoint, m_goal);
      armController.setSetpoint(m_setpoint.position);
      armMotor
          .setVoltage(armController.calculate(getArmActualAngle()) + ff.calculate(m_setpoint.position, ffVelocity));
      // armMotor2
      //     .setVoltage(armController.calculate(getArmActualAngle()) + ff.calculate(m_setpoint.position, ffVelocity));
      RobotContainer.Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  public boolean isFinsished() {
    // 1.75 is calculated
    return armController.getSetpoint() - armMotor.getEncoder().getPosition() * 360 < 1.75
        && armMotor.getEncoder().getVelocity() < 5;
  }

  public void adjustArmAmpAngle() {
    armController.setSetpoint(Constants.Robot.ampAngle);

    armMotor.setVoltage(
        armController.calculate(Constants.Robot.ampAngle) + ff.calculate(Constants.Robot.ampAngle, ffVelocity));
    // armMotor2.setVoltage(
    //     armController.calculate(Constants.Robot.ampAngle) + ff.calculate(Constants.Robot.ampAngle, ffVelocity));
  }

  public void trapezoidAdjustArmAmpAngle() {
    m_goal = new TrapezoidProfile.State(Constants.Robot.ampAngle, 0);
    m_setpoint = m_profile.calculate(dT, m_setpoint, m_goal);
    armController.setSetpoint(m_setpoint.position);
    armMotor.setVoltage(armController.calculate(m_setpoint.position) + ff.calculate(m_setpoint.position, ffVelocity));
    // armMotor2.setVoltage(armController.calculate(m_setpoint.position) + ff.calculate(m_setpoint.position, ffVelocity));
  }

  public void goToHome() {
    Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, 0);
    armController.setSetpoint(Constants.Robot.initialShoulderAngle);
    armMotor.setVoltage(armController.calculate(Constants.Robot.initialShoulderAngle)
        + ff.calculate(Constants.Robot.initialShoulderAngle, ffVelocity));
    // armMotor2.setVoltage(armController.calculate(Constants.Robot.initialShoulderAngle)
    //     + ff.calculate(Constants.Robot.initialShoulderAngle, ffVelocity));
  }

  public void goToSpecifiedAngle(double angle) {
    angle = angle*(Math.PI/180);
    targetAngle = angle;
    // Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, 0);
    armController.setSetpoint(targetAngle);
    armMotor.setVoltage(armController.calculate(armMotor.getEncoder().getPosition(), targetAngle) + ff.calculate(targetAngle, ffVelocity));

  }

  public void trapezoidGoToHome() {
    m_goal = new TrapezoidProfile.State(Constants.Robot.initialShoulderAngle, 0);
    m_setpoint = m_profile.calculate(dT, m_setpoint, m_goal);
    armController.setSetpoint(m_setpoint.position);
    armMotor.setVoltage(armController.calculate(m_setpoint.position) + ff.calculate(m_setpoint.position, ffVelocity));
    // armMotor2.setVoltage(armController.calculate(m_setpoint.position) + ff.calculate(m_setpoint.position, ffVelocity));
  }

  public void trapezoidSubWooferShot() {
    m_goal = new TrapezoidProfile.State(Constants.Robot.subWooferAngle, 0);
    m_setpoint = m_profile.calculate(dT, m_setpoint, m_goal);
    armController.setSetpoint(m_setpoint.position);
    armMotor.setVoltage(armController.calculate(m_setpoint.position) + ff.calculate(m_setpoint.position, ffVelocity));
    // armMotor2.setVoltage(armController.calculate(m_setpoint.position) + ff.calculate(m_setpoint.position, ffVelocity));
  }

  public void manualControl(double input) {
    if (input > 0 && armMotor.getEncoder().getPosition() >= Constants.Robot.ampAngle)
    {
       armMotor.set(0);
    } else 
      armMotor.set(input);
  }
  public void stay() {
    armController.setSetpoint(armMotor.getEncoder().getPosition());
    armController.setTolerance(0);
    armMotor.set(armController.calculate(getArmActualAngle()+ ff.calculate(m_setpoint.position, ffVelocity)));
    // armMotor2.set(armController.calculate(getArmActualAngle()));
  }

  public void subWooferShot() {
    armController.setSetpoint(Constants.Robot.subWooferAngle);
    armMotor.setVoltage(armController.calculate(getArmActualAngle()) + ff.calculate(getArmDesiredAngle(), ffVelocity));
    // armMotor2.setVoltage(armController.calculate(getArmActualAngle()) + ff.calculate(getArmDesiredAngle(), ffVelocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // SYS ID STUFF BELOW

  Consumer<Measure<Voltage>> drive = (Measure<Voltage> volts) -> {
    armMotor.setVoltage(volts.baseUnitMagnitude());
    // armMotor2.setVoltage(volts.baseUnitMagnitude());
  };
  Consumer<SysIdRoutineLog> log = (SysIdRoutineLog logged) -> {
    logged.motor("motor");
  };

  SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(drive, log, this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
