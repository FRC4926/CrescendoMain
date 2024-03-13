// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoncommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

public class AutonArmCommand extends Command {
  double targetAngle = 0;
  boolean holdAtTarget = false;
  /** Creates a new ArmCommand. */
  public AutonArmCommand(double targetAngle, boolean holdAtTarget) {
    this.targetAngle = targetAngle;
    this.holdAtTarget = holdAtTarget;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.toDegrees(Subsystems.m_armSubsystem.armMotor.getEncoder().getPosition())>20){
      Subsystems.m_armSubsystem.armMotor.set(0);
    }else{
      RobotContainer.Subsystems.m_armSubsystem.goToSpecifiedAngle(targetAngle);
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(holdAtTarget){
      return false;
    }else{
      return Subsystems.m_armSubsystem.isFinished(targetAngle);
    }
    
  }
}
