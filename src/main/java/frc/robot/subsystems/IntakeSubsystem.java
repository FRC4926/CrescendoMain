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
  public CANSparkMax frontLeft = new CANSparkMax(Constants.CAN_IDS.TOP_INTAKE, MotorType.kBrushless);
  public CANSparkMax frontRight = new CANSparkMax(Constants.CAN_IDS.BOTTOM_INTAKE, MotorType.kBrushless);

  public IntakeSubsystem() {

  }

  public void intake() {
    if (!input.get()) {
      frontLeft.set(0.3);
      frontRight.set(0.3);
    } else {
      stop();
    }
  }

  public void stop() {
    frontLeft.set(0);
    frontRight.set(0);
  }

  public void displaySensor() {
    SmartDashboard.putBoolean("color sensor", input.get());
  }

  @Override
  public void periodic() {

  }
}
