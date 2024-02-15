// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.Subsystems;


public class ShooterSubsystem extends SubsystemBase {
  public CANSparkMax lowerMotor = new CANSparkMax(6, MotorType.kBrushless);
  public CANSparkMax upperMotor = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax conveyerMotor = new CANSparkMax(9, MotorType.kBrushless);
  //public PIDController  betterController = new PIDController(0.0012,0.00015,0);
  public PIDController  lowerBetterController = new PIDController(0.0010,0.0001,0);
  public PIDController  upperBetterController = new PIDController(0, 0, 0);
  SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(-0.73956, 0, 0);
  public int tolerance = 30;
  public boolean hasPassed;
  public double targetRPM;
  public boolean reached = false;
  public double inputRPM = 0;
  public boolean override = false;
  Timer timer = new Timer();
  public DigitalInput colorSensor = new DigitalInput(6);
  

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    hasPassed = false;
    override = false;
    lowerMotor.restoreFactoryDefaults();
    upperMotor.restoreFactoryDefaults();

    lowerMotor.setOpenLoopRampRate(0);
    upperMotor.setOpenLoopRampRate(0);

    hasPassed = false;

    lowerMotor.setSmartCurrentLimit(85);
    upperMotor.setSmartCurrentLimit(85);

    lowerBetterController.setTolerance(tolerance);
    upperBetterController.setTolerance(tolerance);
    SmartDashboard.putNumber("targetLowerEffort", 0);
    SmartDashboard.putNumber("targetUpperEffort", 0);
    SmartDashboard.putNumber("targetLowerRPM", 0);
    SmartDashboard.putNumber("targetUpperRPM", 0);
    SmartDashboard.putNumber("upperI", 0.0002);
  }
  public void effortShoot(double lower, double upper) {
    // SmartDashboard.getNumber("targetLowerEffort", lower);
    // SmartDashboard.getNumber("targetUpperEffort", upper);
    lowerMotor.set(lower);
    upperMotor.set(upper);
    // SmartDashboard.putNumber("lowerRPM", lowerRPM());
    // SmartDashboard.putNumber("upperRPM", upperRPM());
  }

  public void changeOverride()
  {
    override = !override;
  }

  public boolean getOverride()
  {
    return override;
  }

  public void setOverride(boolean in)
  {
    override = in;
  }

  public void zeroMotors()
  {
    upperMotor.set(0);
    lowerMotor.set(0);
  }

  public void RPMShoot(double lower, double upper)
  { 
    lowerMotor.set(lowerBetterController.calculate(lowerRPM(), lower));
    upperMotor.set(upperBetterController.calculate(upperRPM(), upper));
  }

  public void feedForwardShoot(double upperVelocity)
  {
    upperMotor.setVoltage(shooterFeedForward.calculate(upperVelocity));
  }

  public void changePassed(boolean in)
  {
    hasPassed = in;
  }

  public boolean getPassed()
  {
    return hasPassed;
  }

  public boolean getColorSensor()
  {
    return colorSensor.get();
  }

  public void fullSend()
  {
    upperMotor.setVoltage(-upperMotor.getBusVoltage()*1.2);
    lowerMotor.setVoltage(-lowerMotor.getBusVoltage()*1.2);
  }
  

  
  public double lowerRPM()
  {
    return lowerMotor.getEncoder().getVelocity();
  }

  public double upperRPM()
  {
    return upperMotor.getEncoder().getVelocity(); 
  }

  public boolean isFinished()
  {
    return ((targetRPM != 0) && (inputRPM >= (targetRPM - tolerance)));
  }

  @Override
  public void periodic() {
    upperBetterController = new PIDController(.001,0.005,0);
    // This method will be called once per scheduler run
    
  }
}
