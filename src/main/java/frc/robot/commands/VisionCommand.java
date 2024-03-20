// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.Robot;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.util.GalacPIDController;

public class VisionCommand extends Command {
  double turnEffort;
  double driveEffort;
  GalacPIDController alignPidController = new GalacPIDController(0.014, 0, 0, 0.15, () -> RobotContainer.Subsystems.m_limelightSubsystem.getX(), 0, 1);
  GalacPIDController alignPidControllerG = new GalacPIDController(0.02, 0, 0, 0, () -> RobotContainer.Subsystems.m_driveSubsystem.getGyroYaw()%360, 30, 3);

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
    SmartDashboard.putNumber("Turn Effort", turnEffort);
    SmartDashboard.putNumber("Predicted Turn Effort", alignPidControllerG.getEffort());
    RobotContainer.Subsystems.m_limelightSubsystem.updateLimelight();
    if (RobotContainer.Controllers.m_driverController.getAButton()) {

        turnEffort = alignPidController.getEffort();
      
      RobotContainer.Subsystems.m_driveSubsystem.drive(0, turnEffort);
      if(Math.abs(Subsystems.m_limelightSubsystem.getX())<3){
        RobotContainer.Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, .8);
        RobotContainer.Controllers.m_driverController.setRumble(RumbleType.kBothRumble, .5);
      }
    }else{
      RobotContainer.Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, .8);
      RobotContainer.Controllers.m_driverController.setRumble(RumbleType.kBothRumble, .5);
    }

    if (RobotContainer.Controllers.m_driverController.getXButton()) {
      
      RobotContainer.Subsystems.m_driveSubsystem.drive(0, -alignPidControllerG.getEffort());
      //Subsystems.m_armSubsystem.goToSpecifiedAngle(0.5);
      if(Math.abs(Subsystems.m_limelightSubsystem.getX())<3){
        RobotContainer.Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, .8);
        RobotContainer.Controllers.m_driverController.setRumble(RumbleType.kBothRumble, .5);
      }
    }else{
      RobotContainer.Controllers.m_operatorController.setRumble(RumbleType.kBothRumble, .8);
      RobotContainer.Controllers.m_driverController.setRumble(RumbleType.kBothRumble, .5);
    }

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
