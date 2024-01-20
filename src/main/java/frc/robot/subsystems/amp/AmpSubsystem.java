package frc.robot.subsystems.amp;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpSubsystem extends SubsystemBase {
    public static class Hardware {
        private Spark ampMotor;

        public Hardware(Spark ampMotor) {
            this.ampMotor = ampMotor;
        }
    }
    // Initializes motors, amp object, and navx
    private Spark m_ampMotor;

    /**
     * Create an instance of AmpSubsystem
     * <p>
     * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
     * <p>
     * 
     * @param ampHardware   Hardware devices required by amp
     */
    public AmpSubsystem(Hardware ampHardware) {
        // Instantiates motors and navx
        this.m_ampMotor = ampHardware.ampMotor;

        // Sets master motors inverted
        m_ampMotor.setInverted(true);
    }

    /**
     * Initialize hardware devices for amp subsystem
     * 
     * @return hardware object containing all necessary devices for this subsystem
     */
    public static Hardware initializeHardware() {
        Hardware ampHardware = new Hardware(
            new Spark(Constants.AmpHardware.AMP_MOTOR_ID, MotorKind.NEO)
        );
        return ampHardware;
    }

    /**
     * Stop amp
     */
    private void stop() {
        m_ampMotor.stopMotor();
    }

    //resets encoders
    public void resetEncoders() {
        m_ampMotor.resetEncoder();
    }

    // Controls the robot during teleop
    private void teleop(double speed, double turn) {
        m_ampMotor.set(speed, ControlType.kDutyCycle, -turn, ArbFFUnits.kPercentOut);
    }

    //do we need the turn request for the amp subsystem? does the motor need to turn?
    public Command ampCommand(DoubleSupplier speedRequest, DoubleSupplier turnRequest) {
        return run(() -> teleop(speedRequest.getAsDouble(), turnRequest.getAsDouble()));
    }

    // chan and nim and lyd are awesome
    public Command stopCommand() {
        return run(() -> stop());
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation (for tests)
    }
}
