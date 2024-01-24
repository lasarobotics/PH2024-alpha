package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private Spark climberMotor;

    public Hardware(Spark climberMotor) {
    this.climberMotor = climberMotor;
    }
  }

  private Spark m_climberMotor;

  /**
   * Create an instance of Climber Subsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param climbHardware Hardware devices required by climb
   */
  public ClimberSubsystem(Hardware climberHardware) {
    this.m_climberMotor = climberHardware.climberMotor;
  }

  /**
   * Initialize hardware devices for amp subsystem
   * 
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware climbHardware = new Hardware(
      new Spark(Constants.ClimberHardware.CLIMB_MOTOR_ID, MotorKind.NEO)
    );
    return climbHardware;
  }

  /**
   * Stop climb
   */
  private void stop() {
    m_climberMotor.stopMotor();
  }

  /**
   * Runs the climb motor
   */
  private void climb(double speed) {
    m_climberMotor.set(speed, ControlType.kDutyCycle);
  }

  /**
   * Climb command
   */
  public Command climbCommand(DoubleSupplier speedRequest) {
    return run(() -> climb(speedRequest.getAsDouble()));
  }

  // chan and nim and lyd are awesome
  /**
   * Stop command
   */
  public Command stopCommand() {
    return run(() -> stop());
  }

  @Override
  public void close() {
    m_climberMotor.close();
  }
}
