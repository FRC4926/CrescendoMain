// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Controllers;
import frc.robot.RobotContainer.Subsystems;

public class ShooterCommand extends Command {
  /** Creates a new DriveCommand. */
  public ShooterCommand() {
    addRequirements();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(Controllers.m_operatorController.getYButton()){
    Subsystems.m_shooterSubsystem.shoot();
    Subsystems.m_limelightSubsystem.align();
    Subsystems.m_armSubsystem.adjustArmShooterAngle();
    if(Subsystems.m_limelightSubsystem.isFinsished() && Subsystems.m_armSubsystem.isFinsished() && Subsystems.m_shooterSubsystem.isFinished()){
      Subsystems.m_intakeSubsystem.runConveryorForShoot();
    }
   } 
   else if(Controllers.m_operatorController.getAButton()){
    Subsystems.m_shooterSubsystem.shootAmp();
    Subsystems.m_armSubsystem.adjustArmAmpAngle();
    if(Subsystems.m_armSubsystem.isFinsished()&& Subsystems.m_shooterSubsystem.isFinished()){
      Subsystems.m_intakeSubsystem.runConveryorForShoot();
    }
   }
   else if(Controllers.m_operatorController.getRightTriggerAxis()>0.1){
    Subsystems.m_shooterSubsystem.shoot();
   }
   else{
    Subsystems.m_shooterSubsystem.idle();
    Subsystems.m_armSubsystem.goToHome();
    Subsystems.m_intakeSubsystem.stop();
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