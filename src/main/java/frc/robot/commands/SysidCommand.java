// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkMaxLimitSwitch.Direction;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer.Controllers;
import frc.robot.RobotContainer.Subsystems;

public class SysidCommand extends Command {
  /** Creates a new SysidCommand. */
  public SysidCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Controllers.m_driverController.getAButton())
      Subsystems.m_armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward);
    else if (Controllers.m_driverController.getBButton())
      Subsystems.m_armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse);
    
    if (Controllers.m_driverController.getXButton())
      Subsystems.m_armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
    else if (Controllers.m_driverController.getYButton())
      Subsystems.m_armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
  }

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
