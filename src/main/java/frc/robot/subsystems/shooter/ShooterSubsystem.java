// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.FeedbackSensor;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private Spark flywheelMotor, indexerMotor;

    public Hardware(Spark flywheelMotor, Spark indexerMotor) {
      this.flywheelMotor = flywheelMotor;
      this.indexerMotor = indexerMotor;
    }
  }

  private Spark m_shooterMotor, m_indexerMotor;
  private Measure<Velocity<Angle>> m_flywheelSpeed;
  private Measure<Dimensionless> m_intakeSpeed;
  private Measure<Dimensionless> m_spitSpeed;

  private SparkPIDConfig m_flywheelConfig;

  /**
   * Create an instance of ShooterSubsystem
   * @param shooterHardware Hardware devices needed for Shooter
   * @param flywheelConfig Flywheel PID config
   * @param shooterSpeed Speed for shooting notes
   * @param intakeSpeed Speed for intaking notes
   * @param spitSpeed Speed for spitting out notes
   */
  public ShooterSubsystem(Hardware shooterHardware, SparkPIDConfig flywheelConfig, Measure<Velocity<Angle>> flywheelSpeed, Measure<Dimensionless> intakeSpeed, Measure<Dimensionless> spitSpeed) {
    this.m_shooterMotor = shooterHardware.flywheelMotor;
    this.m_indexerMotor = shooterHardware.indexerMotor;
    this.m_flywheelConfig = flywheelConfig;
    this.m_flywheelSpeed = flywheelSpeed;
    this.m_intakeSpeed = intakeSpeed;
    this.m_spitSpeed = spitSpeed;

    m_shooterMotor.initializeSparkPID(m_flywheelConfig, FeedbackSensor.NEO_ENCODER);
  }

  /**
   * Initialize hardware devices for shooter subsystem
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    return new Hardware(
      new Spark(Constants.ShooterHardware.FLYWHEEL_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.ShooterHardware.INDEXER_MOTOR_ID, MotorKind.NEO)
    );
  }

  /**
   * Shoot disc out from robot
   */
  private void shoot() {
    m_shooterMotor.set(m_flywheelSpeed.in(Units.RPM), ControlType.kVelocity);
  }

  private void shootManual(double x){
    m_shooterMotor.set(x, ControlType.kVelocity);
  }

  /**
   * Spit out disc from robot (shooting at slower speed)
   */
  private void spit() {
    m_shooterMotor.set(+m_spitSpeed.in(Units.Percent), ControlType.kDutyCycle);
    m_indexerMotor.set(+m_spitSpeed.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Feeds disc to shooter motor from indexer motor
   */
  private void feed() {
    m_indexerMotor.set(m_intakeSpeed.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Rotates both motors clockwise to intake disc
   */
  private void intake() {
    m_shooterMotor.set(-m_intakeSpeed.in(Units.Percent), ControlType.kDutyCycle);
    m_indexerMotor.set(-m_intakeSpeed.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Stops both motors
   */
  private void stop() {
    m_shooterMotor.stopMotor();
    m_indexerMotor.stopMotor();
  }

  /**
   * Check if the RPM of shooter motor is within tolerance of the desired RPM
   * @return True if the flywheel is near the desired speed
   */
  public boolean isFlyWheelAtSpeed() {
    return Math.abs(m_shooterMotor.getInputs().encoderVelocity - m_flywheelSpeed.in(Units.RPM)) <= m_flywheelConfig.getTolerance();
  }

  /**
   * Shoot note when flywheel is up to speed
   * @return Command that shoots note
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
   * Shoot command
   * 
   * @return Command which checks if fly wheel is at speed, feeds to shooter motor, and shoots
   */
  public Command shootManualCommand(DoubleSupplier speed) {
    return Commands.sequence(
      runOnce(() -> shootManual(speed.getAsDouble())),
      run(() -> {
        if (isFlyWheelAtSpeed()) feed();
      })
    ).finallyDo(() -> stop());
  }

  

  /**
   * Spit out note at a slower speed
   * @return Command that runs both motors to shoot at a slower speed
   */
  public Command spitCommand() {
    return startEnd(() -> spit(), () -> stop());
  }

  /**
   * Intake note command
   * @return Command that intakes note
   */
  public Command intakeCommand() {
    return startEnd(() -> intake(), () -> stop());
  }

  /**
   * Stop command
   * @return Command that stops both motors
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


  @Override
  public void close() {
    m_shooterMotor.close();
    m_indexerMotor.close();
  }
}
