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
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class ShooterTest {
    
    private final double DELTA = 5e-3;
    private final double MAX_MOTOR_OUTPUT = 1.0;
    private ShooterSubsystem m_shooterSubsystem;
    private DriveSubsystem.Hardware m_drivetrainHardware;

    private Spark m_MasterMotor;
    private Spark m_SlaveMotor;

    private NavX2 m_navx;

    @BeforeEach

    public void setup() {
        m_MasterMotor = mock(Spark.class);
        m_SlaveMotor = mock(Spark.class);
        

    }
    @AfterEach
    public void close() {

    }


}
