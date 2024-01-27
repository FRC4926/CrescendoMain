// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.util.LookUpTableCurrentLimits;

public class DriveSubsystem extends SubsystemBase {
  public AHRS navX = new AHRS(Port.kMXP);
  public CANSparkMax frontLeftMotor = new CANSparkMax(Constants.CAN_IDS.FRONT_LEFT_DRIVE, MotorType.kBrushless);
  public CANSparkMax backLeftMotor = new CANSparkMax(Constants.CAN_IDS.BACK_LEFT_DRIVE, MotorType.kBrushless);
  public CANSparkMax frontRightMotor = new CANSparkMax(Constants.CAN_IDS.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
  public CANSparkMax backRightMotor = new CANSparkMax(Constants.CAN_IDS.BACK_RIGHT_DRIVE, MotorType.kBrushless);
  
  public RelativeEncoder leftEncoder = frontLeftMotor.getEncoder();
  public RelativeEncoder rightEncoder = frontRightMotor.getEncoder();
  
  public DifferentialDrive difDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
  Pose2d m_pose = new Pose2d(0, 0, navX.getRotation2d());
  public DifferentialDriveOdometry m_odometry;

  public Rotation2d getRotation2d() {
    return navX.getRotation2d();
  }

  public double getLeftWheel() {
    return getLeftEncoderVelocity();
  }

  public double getRightWheel() {
    return getRightEncoderVelocity();
  }

  public void resetGyro() {
    navX.reset();
  }

  public double getGyroYaw() {
    return navX.getYaw() % 360;
  }

  public double getGyroPitch() {
    // 0.31 is offset
    return navX.getPitch() % 360;
  }
  public double getGyroRoll(){
    return navX.getRoll();
  }

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() { 
    frontLeftMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    rightEncoder.setPositionConversionFactor(Constants.Robot.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(Constants.Robot.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(Constants.Robot.kLinearDistanceConversionFactor / 60);
    leftEncoder.setVelocityConversionFactor(Constants.Robot.kLinearDistanceConversionFactor / 60);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    frontLeftMotor.enableVoltageCompensation(10);
    frontRightMotor.enableVoltageCompensation(10);

    navX.reset();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), 0, 0);

    setBrakeMode();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  public double getAverageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  public void setBrakeMode() {
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void drive(double forward, double rotate) {
    difDrive.arcadeDrive(forward, rotate);
  }

  public void nullRampRates() {
    backLeftMotor.setOpenLoopRampRate(0);
    frontLeftMotor.setOpenLoopRampRate(0);
    backRightMotor.setOpenLoopRampRate(0);
    frontRightMotor.setOpenLoopRampRate(0);
  }
  
  public double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return leftEncoder.getVelocity();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(navX.getRotation2d(),
        new DifferentialDriveWheelPositions(getLeftEncoderPosition(), getRightEncoderPosition()), pose);
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void setCurrentLimits(int limit) {
    frontLeftMotor.setSmartCurrentLimit(limit);
    frontRightMotor.setSmartCurrentLimit(limit);
    backLeftMotor.setSmartCurrentLimit(limit);
    backRightMotor.setSmartCurrentLimit(limit);
  }

  PIDController speedDriveController = new PIDController(.01, .01, .01);
  double maxWheelSpeed = 4.17576;
  double speed;

  public void speedDrive(double input, double turn) {
    double percent;
    // set left
    if (input - turn > 1)
      percent = 1;
    else if (input - turn < -1)
      percent = -1;
    else
    percent = input - turn;
    speedDriveController.setSetpoint((percent) * maxWheelSpeed);
    speed = speedDriveController.calculate(getLeftWheel());
    frontLeftMotor.set(speed);
    backLeftMotor.set(speed);
    // set right
    if (input + turn > 1)
      percent = 1;
    else if (input + turn < -1)
      percent = -1;
    else
      percent = input + turn;
    speedDriveController.setSetpoint((input + percent) * maxWheelSpeed);
    speed = speedDriveController.calculate(getRightWheel());
    frontRightMotor.set(speed);
    backRightMotor.set(speed);

  }
  public void speedDriveSlowRampUp(double input, double turn) {
    double percent;
    // set left
    if (input - turn > 1)
      percent = 1;
    else if (input - turn < -1)
      percent = -1;
    else
    percent = input - turn;
    speedDriveController.setSetpoint((percent) * maxWheelSpeed);
    speed = speedDriveController.calculate(getLeftWheel());
    frontLeftMotor.set(speed);
    backLeftMotor.set(speed);
    // set right
    if (input + turn > 1)
      percent = 1;
    else if (input + turn < -1)
      percent = -1;
    else
      percent = input + turn;
    speedDriveController.setSetpoint((input + percent) * maxWheelSpeed);
    speed = speedDriveController.calculate(getRightWheel());
    frontRightMotor.set(speed);
    backRightMotor.set(speed);

  }

  LookUpTableCurrentLimits leftLimitTable = new LookUpTableCurrentLimits();
  LookUpTableCurrentLimits rightLimitTable = new LookUpTableCurrentLimits();
  LookUpTableCurrentLimits angleLimitTable = new LookUpTableCurrentLimits();
  double previousLeftSpeed = getLeftWheel();
  double previousRightSpeed = getRightWheel();

  public void adjustCurrentLimit() {
    if ((getRightWheel() - previousRightSpeed) / getRightWheel() > 0.02) {
      rightLimitTable.accelerateTable();
    } else if ((getRightWheel() - previousRightSpeed) / getRightWheel() < -0.02) {
      rightLimitTable.decelerateTable();
    }
    if ((getLeftWheel() - previousRightSpeed) / getLeftWheel() > 0.02) {
      leftLimitTable.accelerateTable();
    } else if ((getLeftWheel() - previousRightSpeed) / getLeftWheel() < -0.02) {
      leftLimitTable.decelerateTable();
    }
    frontLeftMotor.setSmartCurrentLimit((int) (leftLimitTable.lookUpLimit((getLeftWheel())
        - angleLimitTable.lookUpLimit(RobotContainer.Subsystems.m_shooterSubsystem.getCurrentAngle()))));
    backLeftMotor.setSmartCurrentLimit((int) (leftLimitTable.lookUpLimit((getLeftWheel())
        - angleLimitTable.lookUpLimit(RobotContainer.Subsystems.m_shooterSubsystem.getCurrentAngle()))));
    frontRightMotor.setSmartCurrentLimit((int) (leftLimitTable.lookUpLimit((getRightWheel())
        - angleLimitTable.lookUpLimit(RobotContainer.Subsystems.m_shooterSubsystem.getCurrentAngle()))));
    backRightMotor.setSmartCurrentLimit((int) (leftLimitTable.lookUpLimit((getRightWheel())
        - angleLimitTable.lookUpLimit(RobotContainer.Subsystems.m_shooterSubsystem.getCurrentAngle()))));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(rightVolts);
    difDrive.feed();
  }

  @Override
  public void periodic() {
    // if (driverControlled) {
    //   adjustCurrentLimit();
    // }

    m_odometry.update(navX.getRotation2d(), getLeftEncoderPosition(),
    getRightEncoderPosition());
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("frontLeftEncoder", getLeftEncoderPosition());

    SmartDashboard.putNumber("backLeftEncoder", getLeftEncoderPosition());

    SmartDashboard.putNumber("frontRightEncoder", getRightEncoderPosition());

    SmartDashboard.putNumber("backRightEncoder", getRightEncoderPosition());
    SmartDashboard.putNumber("Average Distance", getAverageEncoderDistance());
    SmartDashboard.putNumber("Gyro Yaw", getGyroYaw());
    SmartDashboard.putNumber("Turn Rate", getTurnRate());

    // This method will be called once per scheduler
  }

  public Trajectory getTrajectory(String filePath) {

    try {
      Path path = Filesystem.getDeployDirectory().toPath().resolve("paths/" + filePath + ".wpilib.json");
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);
      return trajectory;
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + ex.getMessage(), ex.getStackTrace());
    }

    return null;
  }

  public RamseteCommand getRamseteCommand(Trajectory trajectory) {

     
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        Subsystems.m_driveSubsystem::getPose,
        new RamseteController(),
        new SimpleMotorFeedforward(
            Constants.Robot.ksVolts,
            Constants.Robot.kvVoltSecondsPerMeter,
            Constants.Robot.kaVoltSecondsSquaredPerMeter),
        Constants.Robot.kDriveKinematics,
        Subsystems.m_driveSubsystem::getWheelSpeeds,
        new PIDController(Constants.Robot.kPDriveVel, 0, 0),
        new PIDController(Constants.Robot.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        Subsystems.m_driveSubsystem::tankDriveVolts,
        Subsystems.m_driveSubsystem);

    return ramseteCommand;
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
