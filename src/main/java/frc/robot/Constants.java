// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
  }

  public static class DriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/LeftFront/Drive", 2);
    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/LeftRear/Drive", 3);
    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/RightFront/Drive", 4);
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/RightRear/Drive", 5);
  }
}
