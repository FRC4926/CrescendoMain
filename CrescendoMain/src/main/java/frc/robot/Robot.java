// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.Commands;
import frc.robot.RobotContainer.Controllers;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.commands.DriveCommand;
//import frc.robot.commands.IntakeCommand;

/**
 * The VM is configured to automatically run this class, and to call the/
 * functions corresponding top
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  // private PowerDistribution pd;
  private RobotContainer m_robotContainer;

  // DoubleLogEntry IntakeVelocityLog;
  // DoubleLogEntry currentLog;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  AddressableLED m_led = new AddressableLED(4);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

  // AddressableLED m_led2;
  // AddressableLEDBuffer m_ledBuffer2 = new AddressableLEDBuffer(60);
  // Timer timer = new Timer();
  // boolean isGreen = false;
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_led.setLength(m_ledBuffer.getLength());

    // // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    // m_led2 = new AddressableLED(9);

    // m_ledBuffer2 = new AddressableLEDBuffer(61);
    // m_led2.setLength(m_ledBuffer.getLength());

    // // Set the data
    // m_led2.setData(m_ledBuffer);
    // m_led2.start();

    // timer.start();
    m_robotContainer = new RobotContainer();

    

    // DataLogManager.start();
    // DataLog log = DataLogManager.getLog();
    // IntakeVelocityLog = new DoubleLogEntry(log, "Intake Velocity");
    // currentLog = new DoubleLogEntry(log, "Current");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Subsystems.m_armSubsystem.armMotor.setIdleMode(IdleMode.kCoast);
    SmartDashboard.putNumber("AngleD", Subsystems.m_armSubsystem.armMotor.getEncoder().getPosition() * (180 / Math.PI));
    Subsystems.m_limelightSubsystem.updateLimelight();
    SmartDashboard.putNumber("Y", Subsystems.m_limelightSubsystem.y);
    SmartDashboard.putBoolean("color Sensor", Subsystems.m_shooterSubsystem.getColorSensor());
    SmartDashboard.putNumber("Lime Dist", Subsystems.m_limelightSubsystem.calcVerticalDistance());
    
    if (Controllers.m_driverController.getBButtonReleased()) {

      Subsystems.m_shooterSubsystem.toggleChannel(false);

    } else {
      Subsystems.m_shooterSubsystem.toggleChannel(true);

    }

    // if (timer.get() % 1 < 0.02) {
    // isGreen = !isGreen;
    // }
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if (Subsystems.m_shooterSubsystem.getPassed()) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
      } else {
        m_ledBuffer.setRGB(i, 100, 0, 100);
      }

    }
    m_led.setData(m_ledBuffer);
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

   // SmartDashboard.putString("Alliance", DriverStation.getAlliance().get().toString());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // Subsystems.m_driveSubsystem.setBrakeMode();
  }

  @Override
  public void disabledPeriodic() {
    // Subsystems.m_shooterSubsystem.toggleChannel(false);
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    Subsystems.m_driveSubsystem.resetGyro();
    Subsystems.m_driveSubsystem.resetEncoders();
    Subsystems.m_driveSubsystem.setBrakeMode();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("top", Subsystems.m_shooterSubsystem.upperRPM());
    SmartDashboard.putNumber("GetX", Subsystems.m_driveSubsystem.getOdometry().getPoseMeters().getX());

    SmartDashboard.putNumber("GetY", Subsystems.m_driveSubsystem.getOdometry().getPoseMeters().getY());
  }

  @Override
  public void teleopInit() {
    Subsystems.m_armSubsystem.armMotor.setIdleMode(IdleMode.kBrake);
    Subsystems.m_climberSubsystem.resetEncoders();
    Subsystems.m_armSubsystem.resetTimer();

    Subsystems.m_armSubsystem.armMotor.getEncoder().setPosition(-30.9 * (Math.PI / 180));
    // RobotContainer.Subsystems.m_intakeSubsystem.flashlight(true);
    // Subsystems.m_driveSubsystem.driverControlled = true;
    // Subsystems.m_driveSubsystem.nullRampRates();
    Subsystems.m_driveSubsystem.setCurrentLimits(35);
    //Subsystems.m_climberSubsystem.resetEncoders();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Subsystems.m_armSubsystem.armMotor.getEncoder().setPosition(Constants.Robot.initialShoulderAngle-Constants.Robot.slopOffset);
    Subsystems.m_driveSubsystem.resetEncoders();
    Subsystems.m_driveSubsystem.resetGyro();
    Subsystems.m_shooterSubsystem.changePassed(false);
    // Subsystems.m_driveSubsystem.nullRampRates();
    Subsystems.m_driveSubsystem.setBrakeMode();
    // RobotContainer.Subsystems.m_driveSubsystem.setCurrentLimits(60);

    // Subsystems.m_driveSubsystem.pidControllerSetUp();

    // Drives Robot
    CommandScheduler.getInstance().schedule(Commands.m_driveCommand);
    // CommandScheduler.getInstance().schedule(Commands.m_armTestCommand);
    CommandScheduler.getInstance().schedule(Commands.m_armTestCommand);
    // CommandScheduler.getInstance().schedule(Commands.m_shooterCommand);
    CommandScheduler.getInstance().schedule(Commands.m_shooterCommand2);

    CommandScheduler.getInstance().schedule(Commands.m_visionCommand);
    CommandScheduler.getInstance().schedule(Commands.m_climberCommand);
  }

 // DigitalInput input = new DigitalInput(0);

  // Rev2mDistanceSensor sensor = new Rev2mDistanceSensor(Port.kMXP);
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Subsystems.m_climberSubsystem.climber.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putNumber("Climber Encoder", Subsystems.m_climberSubsystem.climber.getEncoder().getPosition());
    SmartDashboard.putNumber("Input Voltage", Subsystems.m_armSubsystem.ff.calculate(0, 1));
    SmartDashboard.putNumber("PID", Subsystems.m_armSubsystem.armController
        .calculate(Subsystems.m_armSubsystem.armMotor.getEncoder().getPosition(), 0));
    SmartDashboard.putNumber("Target Angle", Subsystems.m_armSubsystem.targetAngle);
    SmartDashboard.putNumber("Angle", Subsystems.m_armSubsystem.armMotor.getEncoder().getPosition());

    // IntakeVelocityLog.append(Subsystems.m_intakeSubsystem.intake.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Effort", Subsystems.m_intakeSubsystem.intakeVel());
    // rpmLog.append(Subsystems.m_driveSubsystem.getAverageRPM());
    // currentLog.append(Subsystems.m_driveSubsystem.getAverageCurrent());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    Subsystems.m_driveSubsystem.setCoastMode();

    Subsystems.m_climberSubsystem.resetEncoders();
    Subsystems.m_climberSubsystem.climber.setIdleMode(IdleMode.kCoast);
    Subsystems.m_shooterSubsystem.zeroMotors();
    Subsystems.m_armSubsystem.armMotor.setIdleMode(IdleMode.kCoast);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
        
    //Subsystems.m_armSubsystem.armMotor.set(0.09);
    //SmartDashboard.putNumber("Climber Encoder", Subsystems.m_climberSubsystem.climber.getEncoder().getPosition());
    // SmartDashboard.putBoolean("detect",
    // Subsystems.m_shooterSubsystem.colorSensor.get());
    // Subsystems.m_intakeSubsystem.intake.set(-.8);
    // Subsystems.m_intakeSubsystem.conveyor.set(.9);
    // Subsystems.m_shooterSubsystem.fullp

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
