package frc.robot.subsystems.amp;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private Spark ampMotor;

    public Hardware(Spark ampMotor) {
      this.ampMotor = ampMotor; 
    }
  }

  private Spark m_ampMotor;
  private Measure<Dimensionless> m_ampSpeed;

  /**
   * Create an instance of AmpSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param ampHardware  Hardware devices required by amp
   */
  public AmpSubsystem(Hardware ampHardware, Measure<Dimensionless> ampSpeed) {
    this.m_ampMotor = ampHardware.ampMotor;
    this.m_ampSpeed = ampSpeed;
  }

  /**
   * Initialize hardware devices for amp subsystem
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware ampHardware = new Hardware(
      new Spark(Constants.AmpHardware.AMP_MOTOR_ID, MotorKind.NEO_550)
    );
    return ampHardware;
  }

  /**
   * Runs the amp motor to score
   */
  private void scoreAmp() {
    m_ampMotor.set(-m_ampSpeed.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Run amp motor to intake
   */
  private void intake() {
    m_ampMotor.set(m_ampSpeed.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Stop amp
   */
  private void stop() {
    m_ampMotor.stopMotor();
  }

  /**
   * Command to score a note in the amp
   * @return Command to score note
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
  public void periodic() { // Soumik is NOT amazing!
    // This method will be called once per scheduler run
    m_ampMotor.periodic();
  }

  @Override
  public void close() {
    m_ampMotor.close();
  }
}
