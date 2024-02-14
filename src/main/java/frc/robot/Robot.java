// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.Commands;
import frc.robot.RobotContainer.Controllers;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  DoubleLogEntry rpmLogFL;
  DoubleLogEntry rpmLogFR;
  DoubleLogEntry rpmLogBL;
  DoubleLogEntry rpmLogBR;

  DoubleLogEntry currentLogFL;
  DoubleLogEntry currentLogFR;
  DoubleLogEntry currentLogBL;
  DoubleLogEntry currentLogBR;

  DoubleLogEntry voltageLogFL;
  DoubleLogEntry voltageLogFR;
  DoubleLogEntry voltageLogBL;
  DoubleLogEntry voltageLogBR;

  DoubleLogEntry busVoltage;
  DoubleLogEntry pdVolatage;
  DoubleLogEntry inputFL;
  DoubleLogEntry outputFL;
  PowerDistribution PD = new PowerDistribution();


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    DataLogManager.start();
    DataLog log = DataLogManager.getLog();
    rpmLogFL = new DoubleLogEntry(log, "RPM-FL");
    rpmLogBL = new DoubleLogEntry(log, "RPM-BL");
    rpmLogFR = new DoubleLogEntry(log, "RPM-FR");
    rpmLogBR = new DoubleLogEntry(log, "RPM-BR");

    currentLogFL = new DoubleLogEntry(log, "Current-FL");
    currentLogBL = new DoubleLogEntry(log, "Current-BL");
    currentLogFR = new DoubleLogEntry(log, "Current-FR");
    currentLogBR = new DoubleLogEntry(log, "Current-BR");
    
    currentLogFL = new DoubleLogEntry(log, "Current-FL");
    currentLogBL = new DoubleLogEntry(log, "Current-BL");
    currentLogFR = new DoubleLogEntry(log, "Current-FR");
    currentLogBR = new DoubleLogEntry(log, "Current-BR");

    voltageLogFL = new DoubleLogEntry(log, "Voltage-FL");
    voltageLogBL = new DoubleLogEntry(log, "Voltage-BL");
    voltageLogFR = new DoubleLogEntry(log, "Voltage-FR");
    voltageLogBR = new DoubleLogEntry(log, "Voltage-BR");

    busVoltage = new DoubleLogEntry(log, "BusVoltage");
    inputFL = new DoubleLogEntry(log, "Input-FL");
    pdVolatage = new DoubleLogEntry(log, "pdVoltage");
    outputFL = new DoubleLogEntry(log, "Output-FL");

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
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putString("Alliance", DriverStation.getAlliance().get().toString());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Subsystems.m_driveSubsystem.setBrakeMode();
  }

  @Override
  public void disabledPeriodic() {
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

  }

  @Override
  public void teleopInit() {
    //    AddressableLED m_led = new AddressableLED(9);
    //   AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    //   m_led.setLength(m_ledBuffer.getLength());

    //   m_led.setData(m_ledBuffer);
    //   m_led.start();
    //   for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    //     // Sets the specified LED to the RGB values for red
    //     m_ledBuffer.setRGB(i, 255, 0, 0);
    //  }
     
    //  m_led.setData(m_ledBuffer);
    // Subsystems.m_driveSubsystem.driverControlled = true;
    //Subsystems.m_driveSubsystem.nullRampRates();
    Subsystems.m_driveSubsystem.setCurrentLimits(35);
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Subsystems.m_driveSubsystem.resetEncoders();
    Subsystems.m_driveSubsystem.resetGyro();
    Subsystems.m_driveSubsystem.nullRampRates();
    Subsystems.m_driveSubsystem.setBrakeMode();
    // RobotContainer.Subsystems.m_driveSubsystem.setCurrentLimits(60);

    // Subsystems.m_driveSubsystem.pidControllerSetUp();

    // Drives Robot
    CommandScheduler.getInstance().schedule(Commands.m_driveCommand);

   CommandScheduler.getInstance().schedule(Commands.m_intakeCommand);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    RobotContainer.Controllers.m_driverController.setRumble(RumbleType.kBothRumble, .1);
    SmartDashboard.putNumber("Effort", Subsystems.m_intakeSubsystem.intake.getOutputCurrent());
    // rpmLogFL.append(Subsystems.m_driveSubsystem.frontLeftMotor.getEncoder().getVelocity()/(Constants.Robot.kLinearDistanceConversionFactor / 60));
    // rpmLogBL.append(Subsystems.m_driveSubsystem.backLeftMotor.getEncoder().getVelocity()/(Constants.Robot.kLinearDistanceConversionFactor / 60));
    // rpmLogFR.append(Subsystems.m_driveSubsystem.frontRightMotor.getEncoder().getVelocity()/(Constants.Robot.kLinearDistanceConversionFactor / 60));
    // rpmLogBR.append(Subsystems.m_driveSubsystem.backRightMotor.getEncoder().getVelocity()/(Constants.Robot.kLinearDistanceConversionFactor / 60));
  
    // currentLogFL.append(Subsystems.m_driveSubsystem.frontLeftMotor.getOutputCurrent());
    // currentLogBL.append(Subsystems.m_driveSubsystem.backLeftMotor.getOutputCurrent());
    // currentLogFR.append(Subsystems.m_driveSubsystem.frontRightMotor.getOutputCurrent());
    // currentLogBR.append(Subsystems.m_driveSubsystem.backRightMotor.getOutputCurrent());
    // voltageLogFL.append(Subsystems.m_driveSubsystem.frontLeftMotor.getBusVoltage() * Subsystems.m_driveSubsystem.frontLeftMotor.getAppliedOutput());
    // voltageLogBL.append(Subsystems.m_driveSubsystem.backLeftMotor.getBusVoltage() * Subsystems.m_driveSubsystem.backLeftMotor.getAppliedOutput());
    // voltageLogFR.append(Subsystems.m_driveSubsystem.frontRightMotor.getBusVoltage() * Subsystems.m_driveSubsystem.frontRightMotor.getAppliedOutput());
    // voltageLogBR.append(Subsystems.m_driveSubsystem.backRightMotor.getBusVoltage() * Subsystems.m_driveSubsystem.backRightMotor.getAppliedOutput());

    // busVoltage.append(Subsystems.m_driveSubsystem.frontLeftMotor.getBusVoltage());
    // pdVolatage.append(PD.getVoltage());
    // inputFL.append(Controllers.m_driverController.getLeftY()*Subsystems.m_driveSubsystem.backRightMotor.getBusVoltage()*1.2);
    // outputFL.append( Subsystems.m_driveSubsystem.frontLeftMotor.getAppliedOutput());
    // inputFR.append(Subsystems.m_driveSubsystem.frontRightMotor.getBusVoltage() * Controllers.m_driverController.getLeftY());
    // inputBR.append(Subsystems.m_driveSubsystem.backRightMotor.getBusVoltage() * Controllers.m_driverController.getLeftY());

    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    Subsystems.m_driveSubsystem.setCoastMode();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
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
