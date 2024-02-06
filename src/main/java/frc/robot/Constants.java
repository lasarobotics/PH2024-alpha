// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.lasarobotics.hardware.ctre.TalonSRX;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.units.Angle;
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
public final class Constants {
  public static class Global {
    public static final int NEO_ENCODER_TICKS_PER_ROTATION = 42;
    public static final int NEO_MAX_RPM = 5880;
    public static final double ROBOT_LOOP_PERIOD = 1.0 / 60.0;    
  }

  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final double CONTROLLER_DEADBAND = 0.15;
  }

  public static class DriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/LeftFront/Drive", 2);
    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/LeftRear/Drive", 3);
    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/RightFront/Drive", 4);
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/RightRear/Drive", 5);
  }

  public static class Shooter {
    public static final Measure<Velocity<Angle>> SHOOTER_SPEED = Units.RPM.of(4000);
    public static final double SPIT_SPEED = 0.5;
    public static final double INTAKE_SPEED = 0.8;
    public static final PIDConstants SHOOTER_PID = new PIDConstants(
      1.5e-4, 
      0.0, 
      1.0e-2, 
      1.7e-4
    );
    
    public static final SparkPIDConfig FLYWHEEL_CONFIG = new SparkPIDConfig(SHOOTER_PID, 
    false, 
    false, 
    20.0
  );
  }

  public static class ShootHardware {
    public static final Spark.ID MASTER_MOTOR_ID = new Spark.ID("ShootHardware/Master/Shoot", 6);
    public static final Spark.ID INDEXER_MOTOR_ID = new Spark.ID("ShootHardware/Indexer/Shoot", 7);
  }

  public static class AmpHardware {
    public static final TalonSRX.ID AMP_MOTOR_ID = new TalonSRX.ID("AmpHardware/AmpMotor", 8);
  }

  public static class ClimberHardware {
    public static final Spark.ID CLIMB_MOTOR_ID = new Spark.ID("ClimberHardware/Climb", 9);
  }
}
