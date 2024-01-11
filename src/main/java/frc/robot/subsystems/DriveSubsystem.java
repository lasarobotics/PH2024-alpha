// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.utils.GlobalConstants;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  public static class Hardware {
    private Spark lMasterMotor, rMasterMotor;
    private Spark lSlaveMotor, rSlaveMotor;
    private NavX2 navx;

    public Hardware(
        Spark lMasterMotor,
        Spark rMasterMotor,
        Spark lSlaveMotor,
        Spark rSlaveMotor,
        NavX2 navx) {
      this.lMasterMotor = lMasterMotor;
      this.rMasterMotor = rMasterMotor;
      this.lSlaveMotor = lSlaveMotor;
      this.rSlaveMotor = rSlaveMotor;
      this.navx = navx;
    }
  }
  // Initializes motors, drivetrain object, and navx
  private Spark m_lMasterMotor, m_lSlaveMotor;
  private Spark m_rMasterMotor, m_rSlaveMotor;

  private NavX2 m_navx;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param drivetrainHardware   Hardware devices required by drivetrain
   */
  public DriveSubsystem(Hardware drivetrainHardware) {
    // Instantiates motors and navx
    this.m_lMasterMotor = drivetrainHardware.lMasterMotor;
    this.m_rMasterMotor = drivetrainHardware.rMasterMotor;
    this.m_lSlaveMotor = drivetrainHardware.lSlaveMotor;
    this.m_rSlaveMotor = drivetrainHardware.rSlaveMotor;

    this.m_navx = drivetrainHardware.navx;

    // Sets master motors inverted
    m_rMasterMotor.setInverted(true);
    m_rSlaveMotor.setInverted(true);

    // Makes slaves follow masters
    m_lSlaveMotor.follow(m_lMasterMotor);
    m_rSlaveMotor.follow(m_rMasterMotor);
  }

  /**
   * Initialize hardware devices for drive subsystem
   * 
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware drivetrainHardware = new Hardware(
      new Spark(Constants.DriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.DriveHardware.LEFT_REAR_DRIVE_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.DriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID, MotorKind.NEO),
      new NavX2(Constants.DriveHardware.NAVX_ID, GlobalConstants.ROBOT_LOOP_HZ)
    );

    return drivetrainHardware;
  }

  // Controls the robot during teleop
  private void teleop(double speed, double turn) {
    m_lMasterMotor.set(speed, ControlType.kDutyCycle, -turn, ArbFFUnits.kPercentOut);
    m_rMasterMotor.set(speed, ControlType.kDutyCycle, +turn, ArbFFUnits.kPercentOut);
  }

  public Command driveCommand(DoubleSupplier speedRequest, DoubleSupplier turnRequest) {
    return run(() -> teleop(speedRequest.getAsDouble(), turnRequest.getAsDouble()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation (for tests)
  }
}
