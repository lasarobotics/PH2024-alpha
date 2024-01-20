package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.beans.Transient;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.robot.subsystems.intake.intakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.stop;

public class intakeSubsystemTest {
    private final double DELTA = 5e-3;
    private final double MAX_MOTOR_OUTPUT = 1.0;
    private intakeSubsystem m_intakeSubsystem;
    private hardware m_intakeHardware;

    private Spark m_intakeMotor;
    private Spark m_variableInclineMotor;

    @BeforeEach
    public void setup() {
        m_intakeMotor = mock(Spark.class);
        m_variableInclineMotor = mock(Spark.class);

        m_intakeHardware = new intakeSubsystem(m_intakeMotor,
                m_variableInclineMotor);

        m_intakeSubsystem = new intakeSubsystem(m_intakeHardware);

    }

    @AfterEach
    public void close() {
        m_intakeSubsystem.stop();
        m_intakeSubsystem = null;
    }

    @test
    @order(1)
    @DisplayName("Testing if intake/variable incline motors can move clockwise")
    public void clockwise() {
        m_intakeSubsystem.angleToShooterCommand(() -> 1.0, () -> 0.0).execute();
        m_intakeSubsystem.intakeCommand(() -> 1.0, () -> 0.0).execute();

        verify(m_variableInclineMotor, times(1)).set(AdditionalMatchers.eq(+MAX_MOTOR_OUTPUT, DELTA),
                ArgumentMatchers.eq(ControlType.kDutyCycle),
                AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
        verify(m_intakeMotor, times(1)).set(AdditionalMatchers.eq(+MAX_MOTOR_OUTPUT, DELTA),
                ArgumentMatchers.eq(ControlType.kDutyCycle),
                AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    }

    @test
    @order(2)
    @DisplayName("Testing if intake/variable incline motors can move counter-clockwise")
    public void counterClockwise() {
        m_intakeSubsystem.angleToShooterCommand(() -> -1.0, () -> 0.0).execute();
        m_intakeSubsystem.intakeCommand(() -> -1.0, () -> 0.0).execute();

        verify(m_variableInclineMotor, times(1)).set(AdditionalMatchers.eq(-MAX_MOTOR_OUTPUT, DELTA),
                ArgumentMatchers.eq(ControlType.kDutyCycle),
                AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
        verify(m_intakeMotor, times(1)).set(AdditionalMatchers.eq(-MAX_MOTOR_OUTPUT, DELTA),
                ArgumentMatchers.eq(ControlType.kDutyCycle),
                AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    }
}
