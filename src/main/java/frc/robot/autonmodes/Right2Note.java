// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonmodes;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.commands.autoncommands.AutonArmCommand;
import frc.robot.commands.autoncommands.AutonConveyorCommand;
import frc.robot.commands.autoncommands.AutonIntakeCommand;
import frc.robot.commands.autoncommands.AutonShooterCommand;
import frc.robot.commands.autoncommands.AutonSlackinatorCommand;
import frc.robot.commands.autoncommands.AutonVisionCommand;

public class Right2Note {

  public static Command getCommand() {

   Trajectory trajectory1 = Subsystems.m_driveSubsystem.getTrajectory("Right2NoteForward");
    Trajectory trajectory2 = Subsystems.m_driveSubsystem.getTrajectory("Right2NoteForward2");
    Trajectory trajectory3 = Subsystems.m_driveSubsystem.getTrajectory("Right2NoteBackward");

    return Commands.runOnce(() -> Subsystems.m_driveSubsystem.resetPose(trajectory1.getInitialPose()))
        .andThen(new AutonShooterCommand(Constants.Auton.subwooferTopRPM,Constants.Auton.subwooferBottomRPM).deadlineWith(new AutonSlackinatorCommand()))
        .andThen(Subsystems.m_driveSubsystem.getRamseteCommand(trajectory1))
        .andThen(Commands.runOnce(() -> Subsystems.m_driveSubsystem.tankDriveVolts(0, 0)))
        .andThen(Commands.waitSeconds(0.2))
        .andThen(new AutonVisionCommand())
        .andThen(new AutonArmCommand(2, false))
        .andThen((Commands.waitSeconds(Constants.Auton.feedTime).deadlineWith(new AutonConveyorCommand())).deadlineWith(new AutonArmCommand(2,true)))
        .andThen(Commands.runOnce(()->Subsystems.m_shooterSubsystem.updateHasPassed()))
        .andThen(Commands.runOnce(()->Subsystems.m_shooterSubsystem.conveyerMotor.set(0)))
        .andThen(Subsystems.m_driveSubsystem.getRamseteCommand(trajectory2)
        .alongWith(Commands.waitSeconds(5).raceWith(new AutonIntakeCommand())).deadlineWith(Commands.runOnce(()->Subsystems.m_armSubsystem.armMotor.set(-0.1))))
        .andThen(Commands.runOnce(() -> Subsystems.m_driveSubsystem.tankDriveVolts(0, 0)))
        .andThen(Subsystems.m_driveSubsystem.getRamseteCommand(trajectory3))
        .andThen(Commands.runOnce(() -> Subsystems.m_driveSubsystem.tankDriveVolts(0, 0)))
        .andThen(Commands.waitSeconds(0.2))
        .andThen(new AutonVisionCommand())
        .andThen(new AutonArmCommand(2, false))
        .andThen((Commands.waitSeconds(Constants.Auton.feedTime).deadlineWith(new AutonConveyorCommand())).deadlineWith(new AutonArmCommand(2,true)))
        .andThen(Commands.runOnce(()->Subsystems.m_shooterSubsystem.updateHasPassed()))
        .andThen(Commands.runOnce(()->Subsystems.m_shooterSubsystem.conveyerMotor.set(0)).deadlineWith(Commands.runOnce(()->Subsystems.m_armSubsystem.armMotor.set(-0.1))));
        

  }
}
