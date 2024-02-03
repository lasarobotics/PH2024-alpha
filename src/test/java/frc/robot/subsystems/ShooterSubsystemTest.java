// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.SparkInputsAutoLogged;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterSubsystemTest {
    private final double DELTA = 5e-3;
    private ShooterSubsystem m_shooterSubsystem;
    private ShooterSubsystem.Hardware m_shooterHardware;

    private Spark m_shooterMotor;
    private Spark m_indexMotor;

    @BeforeEach
    public void setup() {
        m_shooterMotor = mock(Spark.class);
        m_indexMotor = mock(Spark.class);

        when(m_shooterMotor.getKind()).thenReturn(MotorKind.NEO);
        when(m_indexMotor.getKind()).thenReturn(MotorKind.NEO);

        m_shooterHardware = new ShooterSubsystem.Hardware(
            m_shooterMotor,
            m_indexMotor
        );

        m_shooterSubsystem = new ShooterSubsystem(
            m_shooterHardware, 
            Constants.Shooter.SHOOTER_PID, 
            Constants.Shooter.SHOOTER_SPEED, 
            Constants.Shooter.INTAKE_SPEED, 
            Constants.Shooter.SPIT_SPEED
        );
    }

    @AfterEach
    public void close() {
        m_shooterSubsystem.close();
        m_shooterSubsystem = null;
    }

    @Test
    @Order(1)
    @DisplayName("Test if the index motor runs when shooter is at full speed.")
    public void shouldFeed() {
        SparkInputsAutoLogged mock = new SparkInputsAutoLogged();
        mock.encoderVelocity = Constants.Shooter.DESIRED_RPM;
        when(m_shooterMotor.getInputs()).thenReturn(mock);
        m_shooterSubsystem.shootCommand().execute();

        verify(m_indexMotor, times(1)).set(
            AdditionalMatchers.eq(Constants.Shooter.INTAKE_SPEED, DELTA),
            ArgumentMatchers.eq(ControlType.kDutyCycle)
        );

        verify(m_shooterMotor, times(1)).set(
            AdditionalMatchers.eq(Constants.Shooter.SHOOTER_SPEED.in(Units.RPM), DELTA),
            ArgumentMatchers.eq(ControlType.kDutyCycle)
        );
    }

    @Test
    @Order(2)
    @DisplayName("Test if the index motor does NOT run when shooter is not at full speed.")
    public void shouldFeed2() {
        SparkInputsAutoLogged mock = new SparkInputsAutoLogged();
        mock.encoderVelocity = 0;
        when(m_shooterMotor.getInputs()).thenReturn(mock);
        m_shooterSubsystem.shootCommand().execute();

        verify(m_indexMotor, times(0)).set(
            AdditionalMatchers.eq(Constants.Shooter.INTAKE_SPEED, DELTA),
            ArgumentMatchers.eq(ControlType.kDutyCycle)
        );

        verify(m_shooterMotor, times(1)).set(
            AdditionalMatchers.eq(Constants.Shooter.SHOOTER_SPEED.in(Units.RPM), DELTA),
            ArgumentMatchers.eq(ControlType.kDutyCycle)
        );
    }
}
