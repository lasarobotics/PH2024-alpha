// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    SmartDashboard.putNumber("speed", 0);

    // Configure the button bindings
    configureBindings();

    automodeChooser();
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
    PRIMARY_CONTROLLER.x().onTrue(DRIVE_SUBSYSTEM.runOnce(() -> DRIVE_SUBSYSTEM.resetOdometry(new Pose2d())));

    PRIMARY_CONTROLLER.leftBumper().whileTrue(AMP_SUBSYSTEM.intakeCommand());
    PRIMARY_CONTROLLER.rightBumper().whileTrue(AMP_SUBSYSTEM.scoreCommand());

    PRIMARY_CONTROLLER.leftTrigger().whileTrue(SHOOTER_SUBSYSTEM.intakeCommand());
    PRIMARY_CONTROLLER.rightTrigger().whileTrue(SHOOTER_SUBSYSTEM.shootCommand());
    PRIMARY_CONTROLLER.b().whileTrue(SHOOTER_SUBSYSTEM.spitCommand());

    
    PRIMARY_CONTROLLER.a().whileTrue(SHOOTER_SUBSYSTEM.shootManualCommand(() -> SmartDashboard.getNumber("speed", 0.0)));
  }

  private Command rumbleCommand(){
    return Commands.run(() -> {
      PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 0);
    });
  }

  private Command ampIntakeCommand(){
    return Commands.parallel(
      rumbleCommand(),
      AMP_SUBSYSTEM.intakeCommand()
    );
  }

  /**
   * Create dropdown menu with autonomous options
   */
  private void automodeChooser(){
    m_automodeChooser.setDefaultOption("Do nothing", Commands.none());
    m_automodeChooser.addOption("Drive forward", Commands.sequence(
    DRIVE_SUBSYSTEM.driveCommand(() -> 0.5, () -> 0.0)
      .withTimeout(5)
      .andThen(()-> DRIVE_SUBSYSTEM.driveCommand(() -> 0.0, () -> 0.0))
    ));
    m_automodeChooser.addOption("Drive right", Commands.sequence(
    DRIVE_SUBSYSTEM.driveCommand(() -> 0.5, () -> 90.0)
      .withTimeout(5)
      .andThen(()-> DRIVE_SUBSYSTEM.driveCommand(() -> 0.0, () -> 0.0))
    ));
    m_automodeChooser.addOption("Drive left", Commands.sequence(
    DRIVE_SUBSYSTEM.driveCommand(() -> 0.5, () -> -90.0)
      .withTimeout(5)
      .andThen(()-> DRIVE_SUBSYSTEM.driveCommand(() -> 0.0, () -> 0.0))
    ));
    m_automodeChooser.addOption("Stop", Commands.sequence(
      DRIVE_SUBSYSTEM.stopCommand()
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_automodeChooser.getSelected();
  }
}
