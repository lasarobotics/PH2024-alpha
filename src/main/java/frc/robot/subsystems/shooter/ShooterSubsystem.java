// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.hardware.revrobotics.Spark.FeedbackSensor;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.utils.PIDConstants;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private Spark shooterMotor, indexerMotor;

    public Hardware(Spark shooterMotor, Spark indexerMotor) {
      this.shooterMotor = shooterMotor;
      this.indexerMotor = indexerMotor;
    }
  }

  private Spark m_shooterMotor, m_indexerMotor;
  private Measure<Velocity<Angle>> m_shooterSpeed;
  private double m_intakeSpeed;
  private double m_spitSpeed;

  private SparkPIDConfig m_flywheelConfig;

  /**
   * Create an instance of ShooterSubsystem
   * @param shooterHardware Hardware devices needed for Shooter
   * @param pidf PID Constants
   * @shooterSpeed Speed for shooting discs
   * @intakeSpeed Speed for intaking discs
   * @spitSpeed Speed for spitting out discs
   */
  public ShooterSubsystem(Hardware shooterHardware, SparkPIDConfig config, Measure<Velocity<Angle>> shooterSpeed, double intakeSpeed, double spitSpeed) {
    m_shooterMotor = shooterHardware.shooterMotor;
    m_indexerMotor = shooterHardware.indexerMotor;
    m_flywheelConfig = config;
    
    m_shooterMotor.initializeSparkPID(m_flywheelConfig, FeedbackSensor.NEO_ENCODER);

    m_shooterSpeed = shooterSpeed;
    m_intakeSpeed = intakeSpeed;
    m_spitSpeed = spitSpeed;


  }
  
  /**
   * Initialize hardware devices for shooter subsystem
   * 
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initHardware() {
    return new Hardware(
      new Spark(Constants.ShootHardware.MASTER_MOTOR_ID, MotorKind.NEO), 
      new Spark(Constants.ShootHardware.INDEXER_MOTOR_ID, MotorKind.NEO)
    );
  }

  /**
   * Shoot disc out from robot
   */
  private void shoot() {
    m_shooterMotor.set(m_shooterSpeed.in(Units.RPM), ControlType.kVelocity);
  }

  /**
   * Spit out disc from robot (shooting at slower speed)
   */
  private void spit() { 
    m_shooterMotor.set(+m_spitSpeed, ControlType.kDutyCycle);
    m_indexerMotor.set(+m_spitSpeed, ControlType.kDutyCycle);
  }

  /**
   * Feeds disc to shooter motor from indexer motor
   */
  private void feed() {
    System.out.println("feeding");
    m_indexerMotor.set(m_intakeSpeed, ControlType.kDutyCycle);
  }

  /**
   * Rotates both motors clockwise to intake disc
   */
  private void intake() {
    m_shooterMotor.set(-m_intakeSpeed, ControlType.kDutyCycle);
    m_indexerMotor.set(-m_intakeSpeed, ControlType.kDutyCycle);
  }

  /**
   * Stops both motors
   */
  private void stop() {
    m_shooterMotor.stopMotor();
    m_indexerMotor.stopMotor();
  }

  /**
   * Check if the RPM of shooter motor is within 50 RPM of the desired RPM
   * 
   * @return True if the flywheel is near the desired speed.
   */
  public boolean isFlyWheelAtSpeed() {
    return Math.abs(m_shooterMotor.getInputs().encoderVelocity - m_shooterSpeed.in(Units.RPM)) <= m_flywheelConfig.getTolerance();
  }


  @Override
  /**
   * Closes both motors, destroying instance of object
   */
  public void close() {
     m_shooterMotor.close();
     m_indexerMotor.close();
  }
  
  /**
   * Shoot command
   * 
   * @return Command which checks if fly wheel is at speed, feeds to shooter motor, and shoots
   */
  public Command shootCommand() {
    return Commands.sequence(
      runOnce(() -> shoot()),
      run(() -> {
        if (isFlyWheelAtSpeed()) feed();
      })
    ).finallyDo(() -> stop());
  }

  /**
   * Intake command
   */
  public Command intakeCommand() {
    return startEnd(() -> intake(), () -> stop());
  }

  /**
   * Stop command
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_shooterMotor.periodic();
    m_indexerMotor.periodic();
  }

}
