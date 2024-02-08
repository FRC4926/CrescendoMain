// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.util.GalacPIDController;

public class VisionCommand extends Command {
  double turnEffort;
  double driveEffort;
  GalacPIDController alignPidController = new GalacPIDController(0.015, 0, 0, 0.1, () -> RobotContainer.Subsystems.m_limelightSubsystem.getX(), 0, 2);

  /** Creates a new AprilTagAlignmentCommand. */
  public VisionCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.Subsystems.m_limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
