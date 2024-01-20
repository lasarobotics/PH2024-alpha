// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public static class Hardware {
    private Spark rollerMotor;
    private Spark variableInclineMotor;

    public Hardware(Spark rollerMotor, Spark variableInclineMotor) {
      this.rollerMotor = rollerMotor;
      this.variableInclineMotor = variableInclineMotor;
    }
  }

  private Spark m_rollerMotor;
  private Spark m_variableInclineMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(Hardware intakeHardware) {
    this.m_rollerMotor = intakeHardware.rollerMotor;
    this.m_variableInclineMotor = intakeHardware.variableInclineMotor;
  }
  
  /**
   * Initialize hardware devices for intake subsystem
   * 
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware intakeHardware = new Hardware(
      new Spark(Constants.IntakeHardware.ROLLER_MOTOR_ID, MotorKind.NEO), new Spark (Constants.IntakeHardware.VARIABLE_INCLINE_MOTOR_ID, MotorKind.NEO)
    );
    return intakeHardware;
  }

  // Tells the robot to intake
  private void intake(double speed) {
    m_rollerMotor.set(speed, ControlType.kDutyCycle, 0.0, ArbFFUnits.kPercentOut);
  }
  // Tells the robot to incline the platform on which the note is positioned so that the shooter can shoot it
  private void angleToShooter(double angle){
    m_variableInclineMotor.set(angle, ControlType.kDutyCycle, 0.0, ArbFFUnits.kPercentOut);
  }

  // Tells the robot to outtake
  private void outtake(double speed) {
    m_rollerMotor.set(-speed, ControlType.kDutyCycle, 0.0, ArbFFUnits.kPercentOut);
  }

  //Tells the robot to return the intake ramp to original position
  private void angleFromShooter(double angle){
    m_variableInclineMotor.set(-angle, ControlType.kDutyCycle, 0.0, ArbFFUnits.kPercentOut);
  }

  // Stop the robot
  public void stop() {
    m_rollerMotor.stopMotor();;
    m_variableInclineMotor.stopMotor();;
  }
  
  public Command intakeCommand(DoubleSupplier speed) {
    return startEnd(() -> intake(speed.getAsDouble()), () -> stop());
  }

  public Command angleToShooterCommand(DoubleSupplier angle){
    return startEnd(() -> angleToShooter(angle.getAsDouble()), () -> stop());
  }

  public Command outtakeCommand(DoubleSupplier speed) {
    return startEnd(() -> outtake(speed.getAsDouble()), () -> stop());
  }

  public Command angleFromShooterCommand(DoubleSupplier angle){
    return startEnd(() -> angleFromShooter(angle.getAsDouble()), () -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
