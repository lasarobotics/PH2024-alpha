package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

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

public class DriveSubsystemTest {
  private final double DELTA = 5e-3;
  private final double MAX_MOTOR_OUTPUT = 1.0;
  private DriveSubsystem m_driveSubsystem;
  private DriveSubsystem.Hardware m_drivetrainHardware;

  private Spark m_lMasterMotor;
  private Spark m_rMasterMotor;
  private Spark m_lSlaveMotor;
  private Spark m_rSlaveMotor;

  private NavX2 m_navx;

  @BeforeEach
  public void setup() {
    m_lMasterMotor = mock(Spark.class);
    m_lSlaveMotor = mock(Spark.class);
    m_rMasterMotor = mock(Spark.class);
    m_rSlaveMotor = mock(Spark.class);
    m_navx = mock(NavX2.class);

    m_drivetrainHardware = new DriveSubsystem.Hardware(
      m_lMasterMotor,
      m_rMasterMotor,
      m_lSlaveMotor,
      m_rSlaveMotor,
      m_navx
    );

    m_driveSubsystem = new DriveSubsystem(m_drivetrainHardware); 
  }

  @AfterEach
  public void close() {
    m_driveSubsystem.close();
    m_driveSubsystem = null;
  }
  
  @Test
  @Order(1)
  @DisplayName("Test if robot can move forward.")
  public void forward() {
    // Drive forward
    m_driveSubsystem.driveCommand(() -> +1.0, () -> 0.0).execute();
    // Verify that the left and right motors are being driven with the expected values.
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(+MAX_MOTOR_OUTPUT, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(+MAX_MOTOR_OUTPUT, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }
  @Test
  @Order(2)
  @DisplayName("Test if robot can move backward.")
  public void backward() {
    // Drive backward
    m_driveSubsystem.driveCommand(() -> -1.0, () -> 0.0).execute();
    // Verify that the left and right motors are being driven with the expected values.
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(-MAX_MOTOR_OUTPUT, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(-MAX_MOTOR_OUTPUT, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can move left.")
  public void left() {
    // Turn left
    m_driveSubsystem.driveCommand(() -> +0.0, () -> -1.0).execute();
    // Verify that the left and right motors are being driven with the expected values.
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                  AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                  AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot can move right.")
  public void right() {
    // Drive forward
    m_driveSubsystem.driveCommand(() -> +0.0, () -> +1.0).execute();
    // Verify that the left and right motors are being driven with the expected values.
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                  AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                  AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }
}
