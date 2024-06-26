// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

/**
 * Initialize HID constants
 */
public final class Constants {
  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final double CONTROLLER_DEADBAND = 0.15;
  }

  /**
   * Initialize DriveSubsystem hardware
   */
  public static class DriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/LeftFront/Drive", 2);
    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/LeftRear/Drive", 3);
    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/RightFront/Drive", 4);
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/RightRear/Drive", 5);
  }

  /**
   * Initialize Shooter motor speed constants and PID constants
   */
  public static class Shooter {
    public static final Measure<Velocity<Angle>> FLYWHEEL_SPEED = Units.RPM.of(5000);
    public static final Measure<Dimensionless> SPIT_SPEED = Units.Percent.of(50);
    public static final Measure<Dimensionless> INTAKE_SPEED = Units.Percent.of(80);

    public static final SparkPIDConfig FLYWHEEL_CONFIG = new SparkPIDConfig(
      new PIDConstants(3e-4, 0.0, 2.1e-3, 1 / MotorKind.NEO.getMaxRPM()),
      false,
      false,
      60.0
    );
  }

  /**
   * Initalize names for commands in PathPlanner
   */
  public static class NamedCommands {
    public static final String SHOOT_COMMAND_NAME = "shooter";
  }
  /**
   * Initialize names for autos in PathPlanner
   */
  public static class AutoNames {
    public static final String SHOOT = "ShootAuto";
    public static final String LEAVE = "LeaveAuto";
  }

  /**
   * Initialize SmartDashboard names for GUI
   */
  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
    public static final String SMARTDASHBOARD_SHOOTER_SPEED = "Shooter Speed";
  }

  /**
   * Initialize Amp motor speed constants
   */
  public static class Amp {
    public static final Measure<Dimensionless> AMP_SPEED = Units.Percent.of(100);
  }

  /**
   * Initialize Climber motor speed constants
   */
  public static class Climber {
    public static final Measure<Dimensionless> CLIMBER_SPEED = Units.Percent.of(10);
  }

  /**
   * Initialize ShooterSubsystem hardware
   */
  public static class ShooterHardware {
    public static final Spark.ID FLYWHEEL_MOTOR_ID = new Spark.ID("ShooterHardware/Flywheel", 6);
    public static final Spark.ID INDEXER_MOTOR_ID = new Spark.ID("ShooterHardware/Indexer", 7);
  }
  
  /**
   * Initialize AmpSubsystem hardware
   */
  public static class AmpHardware {
    public static final Spark.ID AMP_MOTOR_ID = new Spark.ID("AmpHardware/Amp", 8);
  }

  /**
   * Initialize ClimberSubsystem hardware
   */
  public static class ClimberHardware {
    public static final Spark.ID CLIMB_MOTOR_ID = new Spark.ID("ClimberHardware/Climb", 9);
  }
}
