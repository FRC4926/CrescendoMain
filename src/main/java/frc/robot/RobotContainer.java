// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controller;
import frc.robot.autonmodes.Center1Note;
import frc.robot.autonmodes.Center2Note;
import frc.robot.autonmodes.Center3Note;
import frc.robot.autonmodes.Left2Note;
import frc.robot.autonmodes.Left3Note;
import frc.robot.autonmodes.Left4Note;
import frc.robot.autonmodes.Right2Note;
import frc.robot.autonmodes.Right3Note;
import frc.robot.commands.VisionCommand;
import frc.robot.commands.ArmTestCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveCommand;
//import frc.robot.commands.IntakeCommand;
//import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static class Subsystems {
    public static DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    public final static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    public final static LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
    //public final static VisionSubsystem m_visionSubsystem = new VisionSubsystem();
   // public final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    public final static ArmSubsystem m_armSubsystem = new ArmSubsystem();
    public final static ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  }

  // The Commands are stored here
  public static class Commands {
    public final static ArmTestCommand m_armTestCommand = new ArmTestCommand();
    //public final static ShooterCommand m_shooterCommand = new ShooterCommand();
    public final static DriveCommand m_driveCommand = new DriveCommand();
    public final static VisionCommand m_visionCommand = new VisionCommand();
    public final static ShooterCommand m_shooterCommand2 = new ShooterCommand();
    
    public final static ClimberCommand m_climberCommand = new ClimberCommand();
  }

  // our controllers are stored here
  public static class Controllers {
    // Driver
    public static XboxController m_driverController = new XboxController(
        Controller.kDriverControllerPort);
    // Operator
    public static XboxController m_operatorController = new XboxController(
        Controller.kOperatorControllerPort);
  }

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_chooser.setDefaultOption("Center2Note", Center2Note.getCommand());
    m_chooser.addOption("ShortTaxi", Center1Note.getCommand());
    m_chooser.addOption("Left2Note", Left2Note.getCommand());
    //m_chooser.addOption("Right2Note", Left2Note.getCommand());
    m_chooser.addOption("Left3Note", Left3Note.getCommand());
    //m_chooser.addOption("Left4Note", Left4Note.getCommand());
    //m_chooser.addOption("Center3Note", Center3Note.getCommand());
    //m_chooser.addOption("Right2Note", Right2Note.getCommand());
   // m_chooser.addOption("Right3Note", Right3Note.getCommand());
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_chooser.getSelected();
  }
}
