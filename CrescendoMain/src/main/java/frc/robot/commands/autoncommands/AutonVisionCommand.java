// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoncommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.util.GalacPIDController;

public class AutonVisionCommand extends Command {
  double turnEffort;
  double driveEffort;
  GalacPIDController alignPidController = new GalacPIDController(0.012, 0, 0, .2, () -> RobotContainer.Subsystems.m_limelightSubsystem.getX(), 0, 5);
  /** Creates a new AprilTagAlignmentCommand. */
  public AutonVisionCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.Subsystems.m_limelightSubsystem);
    alignPidController.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("AutonVision",Subsystems.m_limelightSubsystem.getX());
    RobotContainer.Subsystems.m_limelightSubsystem.updateLimelight();
    turnEffort = alignPidController.getEffort();
      RobotContainer.Subsystems.m_driveSubsystem.drive(0, turnEffort);
    }    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Subsystems.m_limelightSubsystem.getX())<5 && RobotContainer.Subsystems.m_driveSubsystem.frontLeftMotor.getEncoder().getVelocity()<.2 && Subsystems.m_limelightSubsystem.getID()==1;
  }
}
