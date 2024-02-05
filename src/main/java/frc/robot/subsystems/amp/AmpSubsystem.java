package frc.robot.subsystems.amp;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.ctre.TalonSRX;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;


import com.revrobotics.CANSparkBase.ControlType;

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

  private TalonSRX m_ampMotor;

  /**
   * Create an instance of AmpSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
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
  private void scoreAmp(double speed) {
    m_ampMotor.set(speed);
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
  public Command scoreAmpCommand(DoubleSupplier speedRequest) {
    return startEnd(() -> scoreAmp(speedRequest.getAsDouble()), () -> stop());
  }

  /**
   * Stop Command
   */
  public Command stopCommand() {  // chan and nim and lyd are awesome
    return runOnce(() -> stop());
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
