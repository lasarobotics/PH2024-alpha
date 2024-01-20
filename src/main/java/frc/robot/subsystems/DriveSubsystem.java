// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.utils.GlobalConstants;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private Spark lMasterMotor, rMasterMotor;
    private Spark lSlaveMotor, rSlaveMotor;
    private NavX2 navx;

    public Hardware(
      
        Spark lMasterMotor,
        Spark rMasterMotor,
        Spark lSlaveMotor,
        Spark rSlaveMotor,
        NavX2 navx) {
      this.lMasterMotor = lMasterMotor;
      this.rMasterMotor = rMasterMotor;
      this.lSlaveMotor = lSlaveMotor;
      this.rSlaveMotor = rSlaveMotor;
      this.navx = navx;
    }
  }

  public static final double DRIVE_TRACK_WIDTH = 0.6858;



  // Initializes motors, drivetrain object, and navx
  private Spark m_lMasterMotor, m_lSlaveMotor;
  private Spark m_rMasterMotor, m_rSlaveMotor;

  private NavX2 m_navx;
  
  //Robot Odometry
  private DifferentialDrivePoseEstimator m_poseEstimator;
  private DifferentialDriveKinematics m_kinematics;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param drivetrainHardware   Hardware devices required by drivetrain
   */
  public DriveSubsystem(Hardware drivetrainHardware) {
    // Instantiates motors and navx
    this.m_lMasterMotor = drivetrainHardware.lMasterMotor;
    this.m_rMasterMotor = drivetrainHardware.rMasterMotor;
    this.m_lSlaveMotor = drivetrainHardware.lSlaveMotor;
    this.m_rSlaveMotor = drivetrainHardware.rSlaveMotor;

    this.m_navx = drivetrainHardware.navx;

    // Sets master motors inverted
    m_rMasterMotor.setInverted(true);
    m_rSlaveMotor.setInverted(true);

    // Makes slaves follow masters
    m_lSlaveMotor.follow(m_lMasterMotor);
    m_rSlaveMotor.follow(m_rMasterMotor);


    //Initialize odometry
     m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics,
        new Rotation2d(),
        0.0,
        0.0,
        new Pose2d());
     m_kinematics = new DifferentialDriveKinematics(DRIVE_TRACK_WIDTH);
  }

  /**
   * Initialize hardware devices for drive subsystem
   * 
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() { 
    Hardware drivetrainHardware = new Hardware(
      new Spark(Constants.DriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.DriveHardware.LEFT_REAR_DRIVE_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.DriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID, MotorKind.NEO),
      new NavX2(Constants.DriveHardware.NAVX_ID, GlobalConstants.ROBOT_LOOP_HZ)
    );

    return drivetrainHardware;
  }

  // Controls the robot during teleop
  private void teleop(double speed, double turn) {
    m_lMasterMotor.set(speed, ControlType.kDutyCycle, -turn, ArbFFUnits.kPercentOut);
    m_rMasterMotor.set(speed, ControlType.kDutyCycle, +turn, ArbFFUnits.kPercentOut);
  }

  //resets encoders
  public void resetEncoders() {
    m_lMasterMotor.resetEncoder();
    m_rMasterMotor.resetEncoder();
  }

  //resets orientation of the robot to default stance
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_poseEstimator.resetPosition(m_navx.getInputs().rotation2d, 0.0, 0.0, pose);
  }

  //updates orientation of robot
  public void updateOdometry() {
    m_poseEstimator.update(Rotation2d.fromDegrees(getAngle()),
        m_lMasterMotor.getInputs().encoderPosition,
        m_rMasterMotor.getInputs().encoderPosition);
  }

  // rachit is awesome
  public double getAngle() {
    return m_navx.getInputs().yawAngle;
  }

  //gets current  
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  //returns current kinematics
  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Command driveCommand(DoubleSupplier speedRequest, DoubleSupplier turnRequest) {
    return run(() -> teleop(speedRequest.getAsDouble(), turnRequest.getAsDouble()));
  }

  /**
   * Stop drivetrain
   */
  public void stop() {
    m_lMasterMotor.stopMotor();
    m_rMasterMotor.stopMotor();
  }


  @Override
  public void close() {
    m_navx.close();
    m_lMasterMotor.close();
    m_rMasterMotor.close();
    m_lSlaveMotor.close();
    m_rSlaveMotor.close();
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