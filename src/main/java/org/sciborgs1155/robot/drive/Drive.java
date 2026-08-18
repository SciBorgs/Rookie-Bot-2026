package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Ports.Drive.LEFT_FOLLOWER;
import static org.sciborgs1155.robot.Ports.Drive.LEFT_LEADER;
import static org.sciborgs1155.robot.Ports.Drive.RIGHT_FOLLOWER;
import static org.sciborgs1155.robot.Ports.Drive.RIGHT_LEADER;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_SPEED;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Ports;
import org.sciborgs1155.robot.drive.DriveConstants.FF;
import org.sciborgs1155.robot.drive.DriveConstants.PID;

public class Drive extends SubsystemBase {

  private final SparkFlex rightLeader = new SparkFlex(RIGHT_LEADER, MotorType.kBrushless);
  private final SparkFlex rightFollower = new SparkFlex(RIGHT_FOLLOWER, MotorType.kBrushless);
  private final SparkFlex leftLeader = new SparkFlex(LEFT_LEADER, MotorType.kBrushless);
  private final SparkFlex leftFollower = new SparkFlex(LEFT_FOLLOWER, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);
  private final PIDController leftPidController = 
    new PIDController(PID.kP, PID.kI, PID.kP);
  private final PIDController rightPIDController = 
    new PIDController(PID.kP, PID.kI, PID.kP);


  private final DifferentialDriveOdometry odometry;

  /* Sets Motor configs  */
  public Drive() {
    SparkFlexConfig globalConfig = new SparkFlexConfig(); // for every motor (universal)
    SparkFlexConfig rightLeaderConfig = new SparkFlexConfig();
    SparkFlexConfig leftLeaderConfig = new SparkFlexConfig();
    SparkFlexConfig leftFollowerConfig = new SparkFlexConfig();
    SparkFlexConfig rightFollowerConfig = new SparkFlexConfig();

    odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());

    globalConfig.idleMode(IdleMode.kBrake);

    leftFollowerConfig.apply(globalConfig).follow(Ports.Drive.LEFT_LEADER);
    rightFollowerConfig.apply(globalConfig).follow(Ports.Drive.RIGHT_LEADER);
    leftLeaderConfig
        .apply(globalConfig)
        .inverted(true)
        .encoder
        .positionConversionFactor(DriveConstants.POSITION_FACTOR)
        .velocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
    rightLeaderConfig
        .apply(globalConfig)
        .inverted(false)
        .encoder
        .positionConversionFactor(DriveConstants.POSITION_FACTOR)
        .velocityConversionFactor(DriveConstants.VELOCITY_FACTOR);

    leftLeader.configure(
        leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    leftFollower.configure(
        leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightLeader.configure(
        rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightFollower.configure(
        rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    gyro.reset();
  }

  /**
   * private method thats sets the speed of the left and right motor
   *
   * @param leftSpeed value from -1 to 1, representing percentages of max speed for left motors
   * @param rightSpeed value from -1 to 1, representing percentages of max speed for right motors
   */
  private void drive(double leftSpeed, double rightSpeed) {

    final double realLeftSpeed = leftSpeed * MAX_SPEED.in(Meters);
    final double realRightSpeed = rightSpeed * MAX_SPEED.in(Meters);

      final double leftFeedforward = feedforward.calculate(realLeftSpeed);
      final double rightFeedforward = feedforward.calculate(realRightSpeed);

      final double leftPID = 
        leftPidController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
      final double rightPID = 
        rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);

      double leftVoltage = leftPID + leftFeedforward;
      double rightVoltage = rightPID + rightFeedforward;

      leftLeader.setVoltage(leftVoltage);
      rightLeader.setVoltage(rightVoltage);
  }

  /**
   * Method that returns a run command that sets the speed of the motors
   *
   * @param vLeft velocity of the left motors from -1 to 1 as a double supplier
   * @param vRight velocity of the left motors from -1 to 1 as a double supplier
   * @return run command that calls drive method
   */
  public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
    return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
  }

  /**
   * Updates odometry
   *
   * @param rotation the gyro angle
   */
  private void updateOdometry(Rotation2d rotation) {
    odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  /**
   * Resets Odometry (not sure if this works) for auton 
   *
   * @param rotation the gyro angle
   * @param robotPose robot position as a pose2d
   */
  private void resetOdometry(Rotation2d rotation, Pose2d robotPose) {
    odometry.resetPosition(
        rotation, leftEncoder.getPosition(), rightEncoder.getPosition(), robotPose);
  }

  @Override
  public void periodic() {
    updateOdometry(gyro.getRotation2d());
  }

  /**
   * @return the robot position as Pose2d
   */
  public Pose2d pose() {
    return odometry.getPoseMeters();
  }
}
