package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Ports.Drive.LEFT_FOLLOWER;
import static org.sciborgs1155.robot.Ports.Drive.LEFT_LEADER;
import static org.sciborgs1155.robot.Ports.Drive.RIGHT_FOLLOWER;
import static org.sciborgs1155.robot.Ports.Drive.RIGHT_LEADER;
import static org.sciborgs1155.robot.drive.DriveConstants.DRIVE_MASS;
import static org.sciborgs1155.robot.drive.DriveConstants.GEARING;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_SPEED;
import static org.sciborgs1155.robot.drive.DriveConstants.MOI;
import static org.sciborgs1155.robot.drive.DriveConstants.STD_DEVS;
import static org.sciborgs1155.robot.drive.DriveConstants.TRACK_WIDTH;
import static org.sciborgs1155.robot.drive.DriveConstants.WHEEL_RADIUS;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Ports;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.drive.DriveConstants.FF;
import org.sciborgs1155.robot.drive.DriveConstants.PID;

@Logged
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
    new PIDController(PID.kP, PID.kI, PID.kD);
  private final PIDController rightPIDController = 
    new PIDController(PID.kP, PID.kI, PID.kP);

  private final DifferentialDrivetrainSim driveSim;
  
  private final Field2d field2d = new Field2d();

  private final DifferentialDriveOdometry odometry;

  private final DifferentialDriveKinematics kinematics;

  /* Sets Motor configs  */
  public Drive() {
    SparkFlexConfig globalConfig = new SparkFlexConfig(); // for every motor (universal)
    SparkFlexConfig rightLeaderConfig = new SparkFlexConfig();
    SparkFlexConfig leftLeaderConfig = new SparkFlexConfig();
    SparkFlexConfig leftFollowerConfig = new SparkFlexConfig();
    SparkFlexConfig rightFollowerConfig = new SparkFlexConfig();

    odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());

    driveSim = 
      new DifferentialDrivetrainSim(
        DCMotor.getNeoVortex(2), 
        GEARING, 
        MOI, 
        DRIVE_MASS.in(Kilograms), 
        WHEEL_RADIUS.in(Meters), 
        TRACK_WIDTH.in(Meters), 
        STD_DEVS); //this is the standard deviation of measurment noise for the sensors 

    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
    
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

    final double realLeftSpeed = leftSpeed * MAX_SPEED.in(MetersPerSecond);
    final double realRightSpeed = rightSpeed * MAX_SPEED.in(MetersPerSecond);

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
      driveSim.setInputs(leftVoltage, rightVoltage);

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
   * @param robotPose robot position as a pose2d
   */
  public void resetOdometry(Pose2d robotPose) {
    odometry.resetPosition(
        gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), robotPose);
  }

  @Override
  public void periodic() {
    updateOdometry(Robot.isReal() ? gyro.getRotation2d() : 
        driveSim.getHeading()); //returns roation2D for simulated robot
        field2d.setRobotPose(pose());

  }

  /**
   * @return the robot position as Pose2d
   */
  public Pose2d pose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void simulationPeriodic() {
    // sim.update() tells the simulation how much time has passed
    driveSim.update(Constants.PERIOD.in(Seconds));
    leftEncoder.setPosition(driveSim.getLeftPositionMeters());
    rightEncoder.setPosition(driveSim.getRightPositionMeters());
  }

  /**
   * 
   * @return the robot chasssis speeds 
   */
  public ChassisSpeeds robotRelativeChassisSpeeds() {
    double leftMotorVelocity = leftEncoder.getVelocity();
    double rightMotorVelocity = rightEncoder.getVelocity();
    return kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(leftMotorVelocity, rightMotorVelocity)

    );
  }

  public void setChassisSpeeds(ChassisSpeeds targetSpeed) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(targetSpeed);

    double leftSpeed = wheelSpeeds.leftMetersPerSecond / MAX_SPEED.in(MetersPerSecond);
    double rightSpeed = wheelSpeeds.rightMetersPerSecond / MAX_SPEED.in(MetersPerSecond);

    drive(leftSpeed, rightSpeed);

  }
}
