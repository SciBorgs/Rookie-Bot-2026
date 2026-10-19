package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.Shooting.MINIMUM_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;
import static org.sciborgs1155.robot.shooter.ShooterConstants.IDLE_VELOCITY;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.FieldConstants;
import org.sciborgs1155.robot.commands.shooting.ProjectileVisualizer;
import org.sciborgs1155.robot.commands.shooting.StationaryShooting;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting {

  public static final Translation2d HUB_TARGET =
      FieldConstants.Hub.TOP_CENTER_POINT.toTranslation2d();

  public static final Translation2d LEFT_FEED = FieldConstants.Hub.LEFT_FEED.toTranslation2d();

  public static final DoubleEntry LATENCY_TIME = Tuning.entry("/ShootingData/Latency Time", 0.1);
  private final StationaryShooting algorithm = new StationaryShooting();
  private final Shooter shooter;
  private final Hood hood;
  private final ProjectileVisualizer basketballVisualizer;
  private final Drive drive;

  // for shooting test
  public static final DoubleEntry RADS_TEST = Tuning.entry("/ShootingData/RADS", 100.0);
  public static final DoubleEntry HOOD_DEGREES_TEST =
      Tuning.entry("/ShootingData/Hood Angle", 30.0);

  private Translation2d lastTarget = new Translation2d();

  public Shooting(
      Shooter shooter, Hood hood, ProjectileVisualizer basketballVisualizer, Drive drive) {
    this.shooter = shooter;
    this.hood = hood;
    this.basketballVisualizer = basketballVisualizer;
    this.drive = drive;
  }

  /** rads for fly wheel hood angle drive angle to orientate the tank drive */
  public record ShooterParams(double rads, double hoodAngle, double driveAngle) {}

  /**
   * Calculates the radians for flywheel, hood angle, and drive angle using Moving or Stationary
   * Shot
   *
   * @param target The target goal
   * @return
   */
  public ShooterParams calculateShot(Translation2d target) {

    // predicted robot pose using latency time
    Pose2d latencyPose =
        drive.pose().exp(drive.robotRelativeChassisSpeeds().toTwist2d(LATENCY_TIME.get()));

    lastTarget = target;

    // shifts the hoodPose using latencyPose
    Pose2d hoodPose =
        latencyPose.transformBy(
            new Transform2d(
                CENTER_TO_SHOOTER.getTranslation().toTranslation2d(),
                CENTER_TO_SHOOTER.getRotation().toRotation2d()));

    LoggingUtils.log("/ShootingData/Projected Hood Pose", hoodPose, Pose2d.struct);

    ChassisSpeeds speeds = drive.fieldRelativeChassisSpeeds();

    Vector<N2> hoodSpeeds = VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    // get displacement from hood to from target
    Translation2d hoodTranslation = hoodPose.getTranslation();
    Translation3d displacement = new Translation3d(target.minus(hoodPose.getTranslation()));


    System.out.println("displacement: " + displacement + " hoodSpeeds: " + hoodSpeeds);
    // run shooting alg
    Vector<N3> shotVector = algorithm.calculate(displacement, hoodSpeeds);

    // get x, y, z from vector
    double vx = shotVector.get(0);
    double vy = shotVector.get(1);
    double vz = shotVector.get(2);

    // get the values
    double rads = shotVector.norm();
    double hoodAngle = Math.atan2(vz, Math.hypot(vx, vy));
    double fieldYaw = Math.atan2(vy, vx);
    double targetYaw = fieldYaw;

    LoggingUtils.log("/ShootingData/Distance", hoodTranslation.getDistance(target));

    return new ShooterParams(rads, hoodAngle, targetYaw);
  }

  /**
   * Shoots when stationary
   *
   * @param target The target
   * @return Command that runs the shooting while stationary
   */
  public Command shootNoDriving(Translation2d target) {
    return Commands.waitUntil(
            () ->
                shooter.atVelocity(shooter.setpoint())
                    && shooter.setpoint() > IDLE_VELOCITY.in(RadiansPerSecond)
                    && hood.atPosition(hood.angleSetpoint())
                    && drive.isAligned(calculateShot(target).driveAngle()))
        .andThen(
            Commands.runOnce(
                () -> {
                  if (basketballVisualizer != null) {
                    basketballVisualizer.launchProjectile();
                  }
                }))
        .deadlineFor(runShooterSuperstructure(() -> calculateShot(target)));
  }

  /**
   * Runs the shoter at the rads, hood at the angle, and drive to point at the target angle (for
   * moving while stationary)
   *
   * @param params Shooter Params that contain flywheel rads, hood and drive angle
   * @return Command to run shooter at the rads, hood at angle, and drive at target anlge
   */
  private Command runShooterSuperstructure(Supplier<ShooterParams> params) {
    return Commands.parallel(
        shooter.runShooter(() -> params.get().rads),
        hood.goTo(() -> params.get().hoodAngle),
        Commands.run(() -> drive.pointAtAngle(0, params.get().driveAngle()), drive));
  }

  /**
   * Allows us to test the shooter and hood by manually adjusting vlaues
   *
   * @return Command that runs shooter and hood for manual testing
   */
  public Command shootWithTestData() {
    return shooter
        .runShooter(() -> RADS_TEST.get())
        .alongWith(hood.goTo(() -> HOOD_DEGREES_TEST.get() * Math.PI / 180)) // in radians
        .alongWith(
            basketballVisualizer != null
                ? basketballVisualizer.launchProjectiles()
                : Commands.none());
  }
}
