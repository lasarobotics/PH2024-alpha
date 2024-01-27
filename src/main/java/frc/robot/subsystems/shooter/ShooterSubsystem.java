// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.utils.PIDConstants;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
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


  public ShooterSubsystem(Hardware shooterHardware, PIDConstants pidf, Measure<Velocity<Angle>> shooterSpeed, double intakeSpeed, double spitSpeed) {
    m_shooterMotor = shooterHardware.shooterMotor;
    m_indexerMotor = shooterHardware.indexerMotor;
    m_shooterSpeed = shooterSpeed;
    m_intakeSpeed = intakeSpeed;
    m_spitSpeed = spitSpeed;

  }

  public static Hardware initHardware() {
    return new Hardware(
      new Spark(Constants.ShootHardware.MASTER_MOTOR_ID, MotorKind.NEO), 
      new Spark(Constants.ShootHardware.INDEXER_MOTOR_ID, MotorKind.NEO)
    );
  }

  private void shoot() {
    m_shooterMotor.set(m_shooterSpeed.in(Units.RPM), ControlType.kVelocity);
  }

  private void spit() { 
    m_shooterMotor.set(+m_spitSpeed, ControlType.kDutyCycle);
    m_indexerMotor.set(+m_spitSpeed, ControlType.kDutyCycle);
  }

  private void feed() {
    m_indexerMotor.set(
      m_intakeSpeed, ControlType.kDutyCycle);
  }

  private void intake() {
    m_shooterMotor.set(-m_intakeSpeed, ControlType.kDutyCycle);
    m_indexerMotor.set(-m_intakeSpeed, ControlType.kDutyCycle);
  }
  private void stop() {
    m_shooterMotor.stopMotor();
    m_indexerMotor.stopMotor();
  }

  public boolean isFlyWheelAtSpeed() {
    return Math.abs(m_shooterMotor.getInputs().encoderVelocity - Constants.Shooter.DESIRED_RPM) <= 50;
  }


  @Override
  public void close() {
     m_shooterMotor.close();
     m_indexerMotor.close();
  }

  public Command shootCommand() {
    return Commands.parallel(
      run(() -> shoot()),
      run(() -> {
        if (isFlyWheelAtSpeed()) feed();
      })
    );
  }
  public Command intakeCommand() {
    return startEnd(() -> intake(), () -> stop());
  }

  public Command stopCommand() {
    return run(() -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_shooterMotor.periodic();
    m_indexerMotor.periodic();
  }

}
