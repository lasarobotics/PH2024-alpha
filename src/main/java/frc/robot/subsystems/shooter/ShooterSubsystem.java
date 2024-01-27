// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private Spark masterMotor, slaveMotor;

    public Hardware(Spark masterMotor, Spark slaveMotor) {
      this.masterMotor = masterMotor;
      this.slaveMotor = slaveMotor;
    }
  }

  private Spark m_masterMotor, m_slaveMotor;

  public ShooterSubsystem(Hardware shooterHardware) {
    m_masterMotor = shooterHardware.masterMotor;
    m_slaveMotor = shooterHardware.slaveMotor;

    m_slaveMotor.follow(m_masterMotor);
  }

  public static Hardware initHardware() {
    return new Hardware(
      new Spark(Constants.ShootHardware.MASTER_MOTOR_ID, MotorKind.NEO), 
      new Spark(Constants.ShootHardware.SLAVE_MOTOR_ID, MotorKind.NEO)
    );
  }

  public void shoot(double power) {
    m_masterMotor.set(power, ControlType.kDutyCycle);
  }
  public void intake(double power) {
    m_masterMotor.set(-power, ControlType.kDutyCycle);
  }

  public void stop(){
    m_masterMotor.stopMotor();
  }
  @Override
  public void close() {
     m_masterMotor.close();
     m_slaveMotor.close();
  }

  public Command shootCommand(DoubleSupplier speed) {
    return startEnd(() -> shoot(speed.getAsDouble()), () -> stop());
  }
  public Command intakeCommand(DoubleSupplier speed) {
    return startEnd(() -> intake(speed.getAsDouble()), () -> stop());
  }

  public Command stopCommand() {
    return run(() -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
