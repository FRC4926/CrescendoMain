// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class ShooterSubsystem extends SubsystemBase {
  private PowerDistribution p = new PowerDistribution(12, ModuleType.kRev);
  public CANSparkMax lowerMotor = new CANSparkMax(Constants.CAN_IDS.SHOOTER_BOTTOM, MotorType.kBrushless);
  public CANSparkMax upperMotor = new CANSparkMax(Constants.CAN_IDS.SHOOTER_TOP, MotorType.kBrushless);
  public CANSparkMax conveyerMotor = new CANSparkMax(Constants.CAN_IDS.CONVEYOR, MotorType.kBrushless);
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.CAN_IDS.INTAKE, MotorType.kBrushless);
  //public PIDController  betterController = new PIDController(0.0012,0.00015,0);
  private PIDController  lowerMotorPIDController = new PIDController(0.01,0,0);
  private PIDController  upperMotorPIDController = new PIDController(.01,0,0);
  //SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(-0.73956, 0, 0);
  private boolean hasPassed = false;
  public double targetRPM = 3000;
  private boolean reached = false;
  private double currentRPM = 0;
  private boolean override = false;
  private boolean ampMode = false;
  Timer timer = new Timer();
  private DigitalInput colorSensor = new DigitalInput(Constants.CAN_IDS.COLOR_ID);
  private AnalogInput distanceSensor = new AnalogInput(Constants.CAN_IDS.DISTANCE_ID);
  // AddressableLED m_led = new AddressableLED(4);
  // AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
  

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    hasPassed = false;
    override = false;

    
    // // Set the data
    // m_led.setLength(m_ledBuffer.getLength());
    // m_led.setData(m_ledBuffer);
    // m_led.start();

    // for (var i = 0; i < 15; i++) 
    // {
    //     m_ledBuffer.setRGB(i, 100, 0, 100);
    // }
    // m_led.setData(m_ledBuffer);

    lowerMotor.restoreFactoryDefaults();
    upperMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();
    conveyerMotor.restoreFactoryDefaults();
    conveyerMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.setSmartCurrentLimit(60);
    conveyerMotor.setSmartCurrentLimit(60);

    lowerMotor.setOpenLoopRampRate(0);
    upperMotor.setOpenLoopRampRate(0);

    lowerMotor.setIdleMode(IdleMode.kCoast);
    upperMotor.setIdleMode(IdleMode.kCoast);

    lowerMotor.setSmartCurrentLimit(55);
    upperMotor.setSmartCurrentLimit(55);

    lowerMotorPIDController.setTolerance(Constants.Robot.shooterTolerance);
    upperMotorPIDController.setTolerance(Constants.Robot.shooterTolerance);
    // SmartDashboard.putNumber("targetLowerEffort", 0);
    // SmartDashboard.putNumber("targetUpperEffort", 0);
    // SmartDashboard.putNumber("targetLowerRPM", 0);
    // SmartDashboard.putNumber("targetUpperRPM", 0);

    distanceSensor.setAverageBits(4);

  }

  public boolean getAmpMode(){
    return ampMode;
  }

  public void setAmpMode(boolean ampMode){
    this.ampMode = ampMode;
  }

  public void intake(double effort){
    intakeMotor.set(effort);
  }
  public void convey(double effort){
    conveyerMotor.set(effort);
  }
  public void effortShoot(double lower, double upper) {
    lowerMotor.set(lower);
    upperMotor.set(upper);
  }

  public void changeTargetRPM(double targetRPM){
    this.targetRPM = targetRPM;
  }
 public double getTargetRPM(){
    return targetRPM;
  }
  public void changeOverride()
  {
    override = !override;
  }

  public boolean getOverride()
  {
    return override;
  }

  public void toggleChannel(boolean in)
  {
    p.setSwitchableChannel(in);
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
    lowerMotor.setVoltage(lowerMotorPIDController.calculate(lowerMotor.getEncoder().getVelocity(),lower));
    upperMotor.setVoltage(upperMotorPIDController.calculate(upperMotor.getEncoder().getVelocity(),upper));
  }

  public void changePassed(boolean in)
  {
    hasPassed = in;
    // for (var i = 0; i < 15; i++) {
    //   if (getPassed()) {
    //     m_ledBuffer.setRGB(i, 0, 255, 0);
    //   } else {
    //     m_ledBuffer.setRGB(i, 100, 0, 100);
    //   }
    // }
    // m_led.setData(m_ledBuffer);
  }

  public boolean getPassed()
  {
    return hasPassed;
  }

  public boolean getLimitSwitch()
  {
    return colorSensor.get();

  }

  public boolean distanceSensorTriggered()
  {
    return distanceSensor.getAverageVoltage()>0.35;
  }

  public void fullSend()
  {
    upperMotor.setVoltage(-upperMotor.getBusVoltage()*1.2);
    lowerMotor.setVoltage(-lowerMotor.getBusVoltage()*1.2);
  }

  public boolean isFinishedAuton(double target)
  {
    return (Math.abs(currentRPM) >= Math.abs((target - Constants.Robot.shooterTolerance)));
  }
  
  public boolean isFinished()
  {
    return (Math.abs(currentRPM) >= Math.abs((targetRPM - Constants.Robot.shooterTolerance)));
  }
  public boolean isFinished(double offset)
  {
    return (Math.abs(currentRPM)-offset >= Math.abs((targetRPM - Constants.Robot.shooterTolerance)));
  }
  public boolean isFinishedAuton() {
    return (Math.abs(currentRPM) >= Math.abs((Constants.Auton.subwooferTopRPM - Constants.Robot.shooterTolerance)));
  }

  public void updateHasPassed(){
    hasPassed = colorSensor.get();

  }
  public double lowerRPM()
  {
    return lowerMotor.getEncoder().getVelocity();
  }

  public void updateLEDs()
  {

  }

  public double upperRPM()
  {
    return upperMotor.getEncoder().getVelocity(); 
  }

  @Override
  public void periodic() {
    currentRPM = lowerRPM();
    //upperBetterController = 
    // SmartDashboard.putNumber("Upper RPM", currentRPM);
    // SmartDashboard.putNumber("Lower RPM", lowerRPM());
    if(ampMode){
      targetRPM = 1000;
    }else{
      targetRPM = 3000;
    }
    // This method will be called once per scheduler run
    
  }
}
