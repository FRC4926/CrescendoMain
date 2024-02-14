// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.LookUpTableShooterAngles;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  double p = 0.01;
  double i = 0;
  double d = 0;
  public CANSparkMax armMotor1 = new CANSparkMax(Constants.CAN_IDS.ARM_LEFT, MotorType.kBrushless);
  public CANSparkMax armMotor2 = new CANSparkMax(Constants.CAN_IDS.ARM_RIGHT, MotorType.kBrushless);
  public double angle = armMotor1.getEncoder().getPosition() + Constants.Robot.wristAngle;
  public PIDController armController = new PIDController(p, i, d);
  private LookUpTableShooterAngles angles = new LookUpTableShooterAngles();


  public ArmSubsystem() {
    armMotor1.restoreFactoryDefaults();
    armMotor1.getEncoder().setPosition(Constants.Robot.initialShoulderAngle / 360);
    armMotor2.restoreFactoryDefaults();
    armMotor2.getEncoder().setPosition(Constants.Robot.initialShoulderAngle / 360);
  }


  public double getArmDesiredAngle() {
        if(RobotContainer.Subsystems.m_limelightSubsystem.id== 7 || RobotContainer.Subsystems.m_limelightSubsystem.id== 4 )
    return (angles.lookUpAngle(RobotContainer.Subsystems.m_limelightSubsystem.calcVerticalDistance()));
    else
    return -1;
  }


  public double getArmActualAngle() {
    return (armMotor1.getEncoder().getPosition() + armMotor2.getEncoder().getPosition())/(2) * 360
        * Constants.Robot.shoulderGearRation;
  }


  public void adjustArmShooterAngle() {
    if(getArmDesiredAngle() == -1){
      RobotContainer.Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, .5);
    }else{
    armController.setSetpoint(getArmDesiredAngle());
    armMotor1.set(armController.calculate(getArmActualAngle()));
    armMotor2.set(armController.calculate(getArmActualAngle()));
    RobotContainer.Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, 0);
    }
  }
public boolean isFinsished(){
  //1.75 is calculated
  return armController.getSetpoint()-armMotor1.getEncoder().getPosition()*360<1.75 && armMotor1.getEncoder().getVelocity()<5;
}

  public void adjustArmAmpAngle(){
    armController.setSetpoint(Constants.Robot.ampAngle);
    armMotor1.set(armController.calculate(getArmActualAngle()));
    armMotor2.set(armController.calculate(getArmActualAngle()));


  }


  public void goToHome() {
    armController.setSetpoint(0);
    armMotor1.set(armController.calculate(getArmActualAngle()));
    armMotor2.set(armController.calculate(getArmActualAngle()));
  }


  public void manualControl(double input) {
    armMotor1.set(input);
    armMotor2.set(input);
  }
public void subWooferShot(){
    armController.setSetpoint(Constants.Robot.subWooferAngle);
    armMotor1.set(armController.calculate(getArmActualAngle()));
    armMotor2.set(armController.calculate(getArmActualAngle()));
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}



