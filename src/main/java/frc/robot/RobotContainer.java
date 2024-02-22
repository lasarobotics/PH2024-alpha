// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.amp.AmpSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.AutoTrajectory;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(DriveSubsystem.initializeHardware());
  private static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem(
    ShooterSubsystem.initializeHardware(),
    Constants.Shooter.FLYWHEEL_CONFIG,
    Constants.Shooter.FLYWHEEL_SPEED,
    Constants.Shooter.INTAKE_SPEED,
    Constants.Shooter.SPIT_SPEED
  );
  private static final AmpSubsystem AMP_SUBSYSTEM = new AmpSubsystem(
    AmpSubsystem.initializeHardware(), 
    Constants.Amp.AMP_SPEED
  );
  private static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem(
    ClimberSubsystem.initializeHardware(), 
    Constants.Climber.CLIMBER_SPEED
  );

  private final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(Constants.HID.PRIMARY_CONTROLLER_PORT);

  private static final SendableChooser<Command> m_automodeChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set default command
    DRIVE_SUBSYSTEM.setDefaultCommand(
      DRIVE_SUBSYSTEM.driveCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getRightX()
      )
    );
    
    // Register Named Commands
    NamedCommands.registerCommand(Constants.AutoNames.SHOOT, SHOOTER_SUBSYSTEM.shootCommand());
    DRIVE_SUBSYSTEM.configureAutoBuilder();

    // Configure the button bindings
    configureBindings();

    //Configure ShuffleBoard
    defaultShuffleboardTab(); 
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    PRIMARY_CONTROLLER.x().onTrue(DRIVE_SUBSYSTEM.runOnce(() -> DRIVE_SUBSYSTEM.resetPose(new Pose2d())));

    PRIMARY_CONTROLLER.leftBumper().whileTrue(AMP_SUBSYSTEM.intakeCommand());
    PRIMARY_CONTROLLER.rightBumper().whileTrue(AMP_SUBSYSTEM.scoreCommand());

    PRIMARY_CONTROLLER.leftTrigger().whileTrue(SHOOTER_SUBSYSTEM.intakeCommand());
    PRIMARY_CONTROLLER.rightTrigger().whileTrue(SHOOTER_SUBSYSTEM.shootCommand());
    PRIMARY_CONTROLLER.b().whileTrue(SHOOTER_SUBSYSTEM.spitCommand());

    PRIMARY_CONTROLLER.a().whileTrue(SHOOTER_SUBSYSTEM.shootManualCommand(() -> SmartDashboard.getNumber(
      Constants.SmartDashboard.SMARTDASHBOARD_SHOOTER_SPEED, 0.0)
    ));
  }

  /**
   * Rumbles both sides of the controller
   */
  private Command rumbleCommand(){
    return Commands.run(() -> {
      PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 1.0);
    }).finallyDo(() -> PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  /**
   * Create dropdown menu with autonomous options
   */
  private void automodeChooser(){
    m_automodeChooser.setDefaultOption("Do nothing", Commands.none());
    m_automodeChooser.addOption(Constants.AutoNames.LEAVE, AutoBuilder.buildAuto(Constants.AutoNames.LEAVE));
    m_automodeChooser.addOption(Constants.AutoNames.SHOOT, AutoBuilder.buildAuto(Constants.AutoNames.SHOOT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_automodeChooser.getSelected();
  }

  /**
   * Configure default Shuffleboard tab
   */
  public void defaultShuffleboardTab() {
    Shuffleboard.selectTab(Constants.SmartDashboard.SMARTDASHBOARD_DEFAULT_TAB);
    automodeChooser();
    SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, m_automodeChooser);
    SmartDashboard.putNumber(Constants.SmartDashboard.SMARTDASHBOARD_SHOOTER_SPEED, 0.0);
  }
}
