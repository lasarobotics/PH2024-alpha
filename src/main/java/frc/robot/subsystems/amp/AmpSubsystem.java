package frc.robot.subsystems.amp;

import org.lasarobotics.hardware.ctre.TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private TalonSRX ampMotor;

    public Hardware(TalonSRX ampMotor) {
      this.ampMotor = ampMotor;
    }
  }

  private final double SPEED = 1.0;
  private TalonSRX m_ampMotor;

  /**
   * Create an instance of AmpSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param ampHardware  Hardware devices required by amp
   */
  public AmpSubsystem(Hardware ampHardware) {
    this.m_ampMotor = ampHardware.ampMotor;
  }

  /**
   * Initialize hardware devices for amp subsystem
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware ampHardware = new Hardware(
      new TalonSRX(Constants.AmpHardware.AMP_MOTOR_ID)
    );
    return ampHardware;
  }


  /**
   * Runs the amp motor to score
   */
  private void scoreAmp() {
    m_ampMotor.set(ControlMode.PercentOutput, +SPEED);
  }

  /**
   * Run amp motor to intake
   */
  private void intake() {
    m_ampMotor.set(ControlMode.PercentOutput, -SPEED);
  }

  /**
   * Stop amp
   */
  private void stop() {
    m_ampMotor.stopMotor();
  }

  /**
   * Command to score a note in the amp
   */
  public Command scoreCommand() {
    return startEnd(() -> scoreAmp(), () -> stop());
  }

  /**
   * Stop Command
   */
  public Command stopCommand() {  // chan and nim and lyd are awesome
    return runOnce(() -> stop());
  }

  /**
   * Intake note into amp holder
   * @return Command to intake note
   */
  public Command intakeCommand() {
    return startEnd(() -> intake(), () -> stop());
  }

  @Override
  public void close() {
    m_ampMotor.close();
  }

  @Override

  public void periodic() { // Soumik is amazing!
    // This method will be called once per scheduler run
    m_ampMotor.periodic();
  }
}
