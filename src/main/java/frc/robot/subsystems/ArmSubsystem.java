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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Controllers;
import frc.robot.RobotContainer.Subsystems;
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
  public ArmFeedforward ff = new ArmFeedforward(0, 1.1, 0, 0);
  public CANSparkMax armMotor = new CANSparkMax(Constants.CAN_IDS.ARM, MotorType.kBrushless);
  public int slackBool = 1;
  public double angle = armMotor.getEncoder().getPosition();
  public PIDController armController = new PIDController(p, i, d);
  private LookUpTableShooterAngles angles = new LookUpTableShooterAngles();
  Timer timer = new Timer();
  public ArmSubsystem() {
 
    armMotor.restoreFactoryDefaults();
    // armMotor2.restoreFactoryDefaults();

    armMotor.setIdleMode(IdleMode.kBrake);
    // armMotor2.setIdleMode(IdleMode.kBrake);

    armMotor.setInverted(true);

    armMotor.getEncoder().setPositionConversionFactor((2*Math.PI)*(0.01));
    // armMotor2.getEncoder().setPositionConversionFactor(Constants.Robot.shoulderGearRatio*360);



    armMotor.setSmartCurrentLimit(55);
    // armMotor2.getEncoder().setPosition(Constants.Robot.initialShoulderAngle);

  }
  public double getAngle(double distance){
    return angles.getAngle(distance);
  }

  public void resetTimer(){
    timer.reset();
    timer.start();
  }
  public boolean slackOver(){
    return timer.get()>1;
  }

  


  public void goToSpecifiedAngle(double angle) {
    if(!(Subsystems.m_limelightSubsystem.getID()==-1)){
        angle = angle*(Math.PI/180);
    
      targetAngle = angle;
    }

    // Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, 0);
    armController.setSetpoint(targetAngle);
    // SmartDashboard.putNumber("Predicted Feedforward", ff.calculate(targetAngle, ffVelocity));
    // SmartDashboard.putNumber("Predicted PID", armController.calculate(armMotor.getEncoder().getPosition(), targetAngle));
   armMotor.setVoltage((armController.calculate(armMotor.getEncoder().getPosition(), targetAngle) + ff.calculate(targetAngle, ffVelocity)));
  }

  public void manualControl(double input) {
    if (input > 0 && armMotor.getEncoder().getPosition() >= Constants.Robot.ampAngle)
    {
       armMotor.set(0);
    } else 
      armMotor.set(input);
  }

  public boolean isFinished(double targetAngle){
    return Math.abs(Math.toDegrees(armMotor.getEncoder().getPosition())-targetAngle)<1.5;
  }
  public void goToHomePositionConstantEffort()
  {
    if(armMotor.getEncoder().getPosition()*180/Math.PI>-10)
    {
      armMotor.set(-0.3);
    }
    else if(armMotor.getEncoder().getPosition()*180/Math.PI>-20){
      armMotor.set(-.1);
    }else{
      armMotor.set(0);
    }
  }
  @Override
  public void periodic() {
    if(slackOver()){
      timer.stop();
    }
    // This method will be called once per scheduler run
  }
}
