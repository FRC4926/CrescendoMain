// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer.Controllers;
import frc.robot.RobotContainer.Subsystems;

public class TestCommand extends Command {
  /** Creates a new TestCommand. */
  public TestCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Controllers.m_driverController.getAButton()){
      Subsystems.m_intakeSubsystem.intake();
      Subsystems.m_shooterSubsystem.shoot();
      if(Subsystems.m_shooterSubsystem.isFinished()){
        Subsystems.m_intakeSubsystem.runConveryorForShoot();
      }
    }
    else{
      Subsystems.m_intakeSubsystem.stop();
      Subsystems.m_shooterSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
