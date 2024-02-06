package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
  public DigitalInput input = new DigitalInput(Constants.CAN_IDS.COLOR_ID);
  public CANSparkMax conveyor = new CANSparkMax(Constants.CAN_IDS.CONVEYOR, MotorType.kBrushless);
  public CANSparkMax intake = new CANSparkMax(Constants.CAN_IDS.INTAKE, MotorType.kBrushless);

  public IntakeSubsystem() {
    intake.setSmartCurrentLimit(60);    intake.setSmartCurrentLimit(60);
    conveyor.setSmartCurrentLimit(60);
  }

  public void intake() {
    // if (!input.get()) {
    //   conveyor.set(0.3);
    //   intake.set(0.3);
    //   RobotContainer.Subsystems.m_shooterSubsystem.shoot(-.1,-.1);
    // } else {
    //   stop();
    // }
        intake.set(-1);

  }
  public void feed(){
    conveyor.set(.2);
  }

  public void stop() {
    conveyor.set(0);
    intake.set(0);
    //RobotContainer.Subsystems.m_shooterSubsystem.rev(0,0);
  }
  public void stopShooter(){
    RobotContainer.Subsystems.m_shooterSubsystem.rev(0,0);
  }
  public void displaySensor() {
    SmartDashboard.putBoolean("color sensor", input.get());
  }

  @Override
  public void periodic() {

  }
}