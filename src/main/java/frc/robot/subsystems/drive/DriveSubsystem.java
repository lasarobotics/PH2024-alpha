// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.utils.GlobalConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  
  // Robot Odometry
  private DifferentialDrivePoseEstimator m_poseEstimator;
  private DifferentialDriveKinematics m_kinematics;

  private static final double DRIVE_WHEEL_DIAMETER_METERS = 0.1524;
  private static final double DRIVETRAIN_EFFICIENCY = 0.92;
  private static final double DRIVE_GEAR_RATIO = 8.45;
  private final double DRIVE_TICKS_PER_METER;
  private final double DRIVE_METERS_PER_TICK;
  private final double DRIVE_METERS_PER_ROTATION;
  private final double DRIVE_MAX_LINEAR_SPEED;

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


    DRIVE_TICKS_PER_METER =
      (GlobalConstants.NEO_ENCODER_TICKS_PER_ROTATION * DRIVE_GEAR_RATIO)
      * (1 / (DRIVE_WHEEL_DIAMETER_METERS * Math.PI));
    DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
    DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * GlobalConstants.NEO_ENCODER_TICKS_PER_ROTATION;
    DRIVE_MAX_LINEAR_SPEED = (m_lMasterMotor.getKind().getMaxRPM() / 60) * DRIVE_METERS_PER_ROTATION * DRIVETRAIN_EFFICIENCY;
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

  /**
   * Controls the robot during teleop
   */
  private void teleop(double speed, double turn) {
    speed = MathUtil.applyDeadband(speed, Constants.HID.CONTROLLER_DEADBAND);
    turn = MathUtil.applyDeadband(turn, Constants.HID.CONTROLLER_DEADBAND);
    
    m_lMasterMotor.set(speed - turn, ControlType.kDutyCycle);
    m_rMasterMotor.set(speed + turn, ControlType.kDutyCycle);
  }

  /**
   * Stop drivetrain
   */
  private void stop() {
    m_lMasterMotor.stopMotor();
    m_rMasterMotor.stopMotor();
  }

  /**
   * Configure AutoBuilder for PathPlannerLib
   */
  public void configureAutoBuilder() {
    AutoBuilder.configureRamsete(
      this::getPose,
      this::resetOdometry,
      this::getChassisSpeeds,
      this::autoDrive,
      new ReplanningConfig(), // Default path replanning config. See the API for the options here
      () -> false,
      this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Call this repeatedly to drive during autonomous
   * @param speeds Calculated chasis speeds
   */
  public void autoDrive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    m_lMasterMotor.set(wheelSpeeds.leftMetersPerSecond / DRIVE_MAX_LINEAR_SPEED, ControlType.kDutyCycle);
    m_rMasterMotor.set(wheelSpeeds.rightMetersPerSecond / DRIVE_MAX_LINEAR_SPEED, ControlType.kDutyCycle);
  }

  /**
   * Reset encoders
   */
  public void resetEncoders() {
    m_lMasterMotor.resetEncoder();
    m_rMasterMotor.resetEncoder();
  }

  /**
   * Resets the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_poseEstimator.resetPosition(m_navx.getInputs().rotation2d, 0.0, 0.0, pose);
  }

  /**
   * Updates orientation of robot
   */
  public void updateOdometry() {
    m_poseEstimator.update(Rotation2d.fromDegrees(getAngle().in(Units.Degrees)),
        m_lMasterMotor.getInputs().encoderPosition,
        m_rMasterMotor.getInputs().encoderPosition);
  }

  // rachit is awesome
  /**
   * Get DriveSubsystem angle as detected by the navX MXP
   * 
   * @return Total accumulated yaw angle
   */
  public Measure<Angle> getAngle() {
    return m_navx.getInputs().yawAngle;
  }

  /**
   * Get robot relative speeds
   * @return Robot relative speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(m_lMasterMotor.getInputs().encoderVelocity, m_rMasterMotor.getInputs().encoderVelocity)
    );
  }

  /**
   * Get estimated robot pose
   * @return Currently estimated robot pose
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns track width of the robot
   * 
   * @return track width in meters
   */
  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * Drive command
   */
  public Command driveCommand(DoubleSupplier speedRequest, DoubleSupplier turnRequest) {
    return run(() -> teleop(speedRequest.getAsDouble(), turnRequest.getAsDouble()));
  }

  public Command runPathb(){
      PathPlannerPath path = PathPlannerPath.fromPathFile("Blue Path");

      return AutoBuilder.followPath(path);
  }

  public Command runPathr(){
      PathPlannerPath path = PathPlannerPath.fromPathFile("Red Path");

      return AutoBuilder.followPath(path);
  }

  public Command pathThroughBoth(){
      PathPlannerPath path = PathPlannerPath.fromPathFile("Full Circle");

      return AutoBuilder.followPath(path);
  }
  /**
   * Stop command
   */
  public Command stopCommand() {
    return run(() -> stop());
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
    m_lMasterMotor.periodic();
    m_rMasterMotor.periodic();
    m_lSlaveMotor.periodic();
    m_rSlaveMotor.periodic();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation (for tests)
  }
}