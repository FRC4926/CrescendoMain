// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer.Controllers;
import frc.robot.RobotContainer.Subsystems;

public class ArmTestCommand extends Command {
  /** Creates a new ArmTestCommand. */
  public ArmTestCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("ArmAAngle", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  if (Controllers.m_operatorController.getAButton()) {
      // if (Subsystems.m_armSubsystem.armMotor.getEncoder().getPosition() <= (Math.PI/3))
      // {
        Subsystems.m_armSubsystem.goToSpecifiedAngle(70);
      // } else 
      // {
      //   Subsystems.m_armSubsystem.armMotor.set(0);
      // }
    } else if (Controllers.m_operatorController.getBButton()) {
      Subsystems.m_armSubsystem.goToSpecifiedAngle(-15);
      //Subsystems.m_armSubsystem.armMotor.set(0);
    }else{
      if(!Subsystems.m_armSubsystem.slackOver() || Controllers.m_operatorController.getYButton()){
        Subsystems.m_armSubsystem.changeSlackBool(0);
        Subsystems.m_armSubsystem.armMotor.set(0.06);
        Subsystems.m_armSubsystem.armMotor.getEncoder().setPosition(-30 * (Math.PI / 180));
      }else{
        Subsystems.m_armSubsystem.goToSpecifiedAngle(-35.9);
      }
    }
  }
      //Subsystems.m_shooterSubsystem.changePassed(Subsystems.m_shooterSubsystem.getColorSensor());
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
