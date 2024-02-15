// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.RobotContainer.Controllers;;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */

  public ShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    Subsystems.m_shooterSubsystem.setOverride(false);
    Subsystems.m_shooterSubsystem.changePassed(false); 
    // SmartDashboard.putNumber("targetLowerEffort", 0);
    // SmartDashboard.putNumber("targetUpperEffort", 0);
    SmartDashboard.putNumber("targetRPM", 0);
    // SmartDashboard.putNumber("ConveyorEffort", 0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!Subsystems.m_shooterSubsystem.getOverride())
    {
      if(Subsystems.m_shooterSubsystem.getColorSensor())
        Subsystems.m_shooterSubsystem.changePassed(true);

      if (Subsystems.m_shooterSubsystem.getPassed())
      {
        Subsystems.m_shooterSubsystem.fullSend();
      } else
      {
        Subsystems.m_shooterSubsystem.RPMShoot(-500, -500);
      }
    }



    //inputRPM = ((-Subsystems.m_shooterSubsystem.upperMotor.getEncoder().getVelocity()) + (-Subsystems.m_shooterSubsystem.lowerMotor.getEncoder().getVelocity()))/2;
    Subsystems.m_shooterSubsystem.inputRPM = ((-Subsystems.m_shooterSubsystem.lowerMotor.getEncoder().getVelocity()));
    SmartDashboard.putBoolean("haspass", Subsystems.m_shooterSubsystem.hasPassed);
    SmartDashboard.putBoolean("color", Subsystems.m_shooterSubsystem.getColorSensor());
    SmartDashboard.putNumber("topRPM", Subsystems.m_shooterSubsystem.upperMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("bottomRPM", Subsystems.m_shooterSubsystem.lowerMotor.getEncoder().getVelocity());
    SmartDashboard.putBoolean("override", Subsystems.m_shooterSubsystem.getOverride());
    //SmartDashboard.putNumber("averageRPM", ((-Subsystems.m_shooterSubsystem.upperMotor.getEncoder().getVelocity()) + (-Subsystems.m_shooterSubsystem.lowerMotor.getEncoder().getVelocity()))/2);
    //targetRPM = SmartDashboard.getNumber("targetRPM", 0);
    Subsystems.m_shooterSubsystem.targetRPM = 4900;

    //if ((targetRPM != 0) && (averageRPM >= (targetRPM - Subsystems.m_shooterSubsystem.tolerance)) && (averageRPM <= (targetRPM + Subsystems.m_shooterSubsystem.tolerance))) 
    if (Subsystems.m_shooterSubsystem.isFinished())
    {
      Subsystems.m_shooterSubsystem.reached = true;
      SmartDashboard.putBoolean("reached", true);
      Controllers.m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
    }
    else
    { 
      Subsystems.m_shooterSubsystem.reached = false;
      Controllers.m_driverController.setRumble(RumbleType.kBothRumble, 0);
      SmartDashboard.putBoolean("reached", Subsystems.m_shooterSubsystem.reached);
    }

    if (Controllers.m_driverController.getXButtonReleased())
    {
      Subsystems.m_shooterSubsystem.changeOverride();
      Subsystems.m_shooterSubsystem.zeroMotors();
    } else if (Controllers.m_driverController.getBButton())
    {
      if (Subsystems.m_shooterSubsystem.getOverride())
      {
        Subsystems.m_shooterSubsystem.fullSend();
      }
    } else if (Controllers.m_driverController.getLeftBumper())
    {
      if (Subsystems.m_shooterSubsystem.getOverride())
      {
        Subsystems.m_shooterSubsystem.conveyerMotor.set(-0.25);
      }
    }
    else if (Controllers.m_driverController.getRightTriggerAxis() > 0.05)
    {
        if (Subsystems.m_shooterSubsystem.getOverride())
        {
          Subsystems.m_shooterSubsystem.conveyerMotor.set(0.5); 
        }
        else if (Controllers.m_driverController.getAButton())
        {
          Subsystems.m_shooterSubsystem.conveyerMotor.set(0.5); 
        }
        else if (Subsystems.m_shooterSubsystem.reached)
        {
          Subsystems.m_shooterSubsystem.conveyerMotor.set(0.5);
        }
        if (!Subsystems.m_shooterSubsystem.getOverride())
        {
          if (Subsystems.m_shooterSubsystem.getColorSensor())
            Subsystems.m_shooterSubsystem.hasPassed = true;
          else
            Subsystems.m_shooterSubsystem.hasPassed = false;
        }
    }
    else if(Controllers.m_driverController.getLeftTriggerAxis() > 0.05)
    {
      if (!Subsystems.m_shooterSubsystem.getOverride())
      {
        if (!Subsystems.m_shooterSubsystem.hasPassed)
          Subsystems.m_shooterSubsystem.conveyerMotor.set(0.3);
        else
          Subsystems.m_shooterSubsystem.conveyerMotor.set(0);
      } else
      {
        Subsystems.m_shooterSubsystem.conveyerMotor.set(0.3);
      }
    } else 
    {
        Subsystems.m_shooterSubsystem.conveyerMotor.set(0);
        if (Subsystems.m_shooterSubsystem.getOverride())
          Subsystems.m_shooterSubsystem.RPMShoot(0, 0);
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