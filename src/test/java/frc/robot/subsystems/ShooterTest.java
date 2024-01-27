package frc.robot.subsystems;

import static org.mockito.Mockito.mock;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;

import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterTest {
 private final double DELTA = 5e-3;
 private final double MAX_MOTOR_OUTPUT = 1.0;
 private ShooterSubsystem m_ShooterSubsystem;
 private Spark m_MasterMotor;
 private Spark m_SlaveMotor;
 private ShooterSubsystem.Hardware m_ShooterHardware;


 @BeforeEach
 public void setup() {
    m_MasterMotor = mock(Spark.class);
    m_SlaveMotor = mock(Spark.class);

    m_ShooterHardware = new ShooterSubsystem.Hardware(m_MasterMotor,m_SlaveMotor);
    m_ShooterSubsystem = new ShooterSubsystem(m_ShooterHardware);
 }
@AfterEach
 public void close() {
    m_ShooterSubsystem.close();
    m_ShooterSubsystem = null;
 }
@Test
@Order(1)
@DisplayName("Test if motors move counterclock-wise")
public void counterClockWise() {
    //Test if motors 
  
}
@Test
@Order(2)
@DisplayName("Test if motors move clockwise") 
public void clockWise() {

    }
}


