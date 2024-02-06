// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import java.sql.Driver;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.autoncommands.AutonConveyorCommand;
import frc.robot.commands.autoncommands.AutonIntakeCommand;
import frc.robot.commands.autoncommands.AutonShooterCommand;
import frc.robot.commands.autoncommands.AutonVisionCommand;

public class Left4Note {

  public static Command getCommand() {

    Trajectory trajectory1 = Subsystems.m_driveSubsystem.getTrajectory("Left3NoteForward");
    Trajectory trajectory2 = Subsystems.m_driveSubsystem.getTrajectory("Left4NoteBackward");
    Trajectory trajectory3 = Subsystems.m_driveSubsystem.getTrajectory("Left4NoteForward2");
    Trajectory trajectory4 = Subsystems.m_driveSubsystem.getTrajectory("Left4NoteBackward2");
    Trajectory trajectory5 = Subsystems.m_driveSubsystem.getTrajectory("Left3NoteForward2");

    return Commands.runOnce(() -> Subsystems.m_driveSubsystem.resetPose(trajectory1.getInitialPose()))
        .andThen(new AutonShooterCommand(Constants.Auton.subwooferTopRPM, Constants.Auton.subwooferBottomRPM))
        // .andThen(Commands.waitSeconds(.5).deadlineWith(new AutonConveyorCommand()))
        .andThen(Commands.runOnce(() -> Subsystems.m_intakeSubsystem.stop()))
        .andThen(Subsystems.m_driveSubsystem.getRamseteCommand(trajectory1).deadlineWith(new AutonIntakeCommand()))
        .andThen(Commands.runOnce(() -> Subsystems.m_driveSubsystem.tankDriveVolts(0, 0)))
        .andThen(Commands.runOnce(() -> Subsystems.m_intakeSubsystem.stop()))
        .andThen(Subsystems.m_driveSubsystem.getRamseteCommand(trajectory2).alongWith(new AutonShooterCommand(0, 0)))
        .andThen(Commands.runOnce(() -> Subsystems.m_driveSubsystem.tankDriveVolts(0, 0)))
        // .andThen(Commands.waitSeconds(.5).deadlineWith(new AutonConveyorCommand()))
        .andThen(Commands.runOnce(() -> Subsystems.m_intakeSubsystem.stop()))
        .andThen((Subsystems.m_driveSubsystem.getRamseteCommand(trajectory3).deadlineWith(new AutonIntakeCommand()))
            .alongWith(new AutonShooterCommand(0, 0)))
        .andThen(Commands.runOnce(() -> Subsystems.m_driveSubsystem.tankDriveVolts(0, 0)))
        .andThen(Commands.runOnce(() -> Subsystems.m_intakeSubsystem.stop()))
        .andThen(Subsystems.m_driveSubsystem.getRamseteCommand(trajectory4))
        .andThen(Commands.runOnce(() -> Subsystems.m_driveSubsystem.tankDriveVolts(0, 0)))
        // .andThen(Commands.waitSeconds(.5).deadlineWith(new AutonConveyorCommand()))
        .andThen((Subsystems.m_driveSubsystem.getRamseteCommand(trajectory5).deadlineWith(new AutonIntakeCommand()))
            .alongWith(new AutonShooterCommand(0, 0)))
        .andThen(Commands.runOnce(() -> Subsystems.m_driveSubsystem.tankDriveVolts(0, 0)))
        .andThen(Commands.runOnce(() -> Subsystems.m_intakeSubsystem.stop()));
    // .andThen(Commands.waitSeconds(.5).deadlineWith(new AutonConveyorCommand()));

  }
}
