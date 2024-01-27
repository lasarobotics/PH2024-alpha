package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.robot.subsystems.amp.AmpSubsystem;
import frc.robot.subsystems.amp.AmpSubsystem.Hardware;

public class AmpSubsystemTests {
    private final double DELTA = 5e-3;
    private final double MAX_MOTOR_OUTPUT = 1.0;
    private AmpSubsystem m_ampSubsystem;
    private Hardware m_ampHardware;
    
    private Spark m_ampMotor;

    @BeforeEach
    public void setup(){
        m_ampMotor = mock (Spark.class);
        m_ampHardware = new Hardware(m_ampMotor);
        m_ampSubsystem = new AmpSubsystem(m_ampHardware);
    }

    @AfterEach
    public void close(){
        m_ampSubsystem.stop();
        m_ampSubsystem = null;
    }

    @Test
    @Order(1)
    @DisplayName("Testing if ampMotor can move clockwise")
    public void clockwise(){
        m_ampSubsystem.ampCommand(() -> +1.0, () -> 0.0).execute();

        verify(m_ampMotor, times(1)).set(AdditionalMatchers.eq(+MAX_MOTOR_OUTPUT, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                    AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    }

    @Test
    @Order(2)
    @DisplayName("testing if ampMotor can move counter-clockwise")
    public void counterClockwise(){
        m_ampSubsystem.ampCommand(() -> -1.0, () -> 0.0).execute();

        verify(m_ampMotor, times(1)).set(AdditionalMatchers.eq(-MAX_MOTOR_OUTPUT, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
        AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    }
}
