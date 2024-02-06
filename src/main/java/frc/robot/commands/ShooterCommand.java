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
    if(RobotContainer.Controllers.m_driverController.y().getAsBoolean()){
    if(Subsystems.m_limelightSubsystem.getID()==7 ||  Subsystems.m_limelightSubsystem.getID()==4){
   RobotContainer.Subsystems.m_shooterSubsystem.rev(.3,.3);
    }
  else if(Subsystems.m_limelightSubsystem.getID()==6 || Subsystems.m_limelightSubsystem.getID()== 5){
   RobotContainer.Subsystems.m_shooterSubsystem.rev(.1,.1);
  }
  else{
       RobotContainer.Subsystems.m_shooterSubsystem.rev(.3,.3);
  }
}
else if(RobotContainer.Controllers.m_driverController.x().getAsBoolean()){
  RobotContainer.Subsystems.m_shooterSubsystem.rev(.1, .1);
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
