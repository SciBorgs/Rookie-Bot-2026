package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.Shooting.MINIMUM_VELOCITY;
import static org.sciborgs1155.robot.FieldConstants.allianceReflect;
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
import org.sciborgs1155.robot.commands.shooting.MovingShot;
import org.sciborgs1155.robot.commands.shooting.StationaryShooting;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting {

  public static final DoubleEntry LATENCY_TIME = Tuning.entry("/ShootingData/Latency Time", 0.1);
  private final MovingShot algorithm = new MovingShot();
  private final Shooter shooter;
  private final Hood hood;
  private final Drive drive;
  private final StationaryShooting stationaryShooting = new StationaryShooting();

  // for shooting test
  public static final DoubleEntry RADS_TEST = Tuning.entry("/ShootingData/RADS", 100.0);
  public static final DoubleEntry HOOD_DEGREES_TEST =
      Tuning.entry("/ShootingData/Hood Angle", 30.0);

  private Translation2d lastTarget = new Translation2d();

  public Shooting(Shooter shooter, Hood hood, Drive drive) {
    this.shooter = shooter;
    this.hood = hood;
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
    // reflects the target
    Translation2d reflectedTarget = allianceReflect(target);

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
    Vector<N2> translationSpeeds =
        VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    Vector<N2> rotationSpeeds =
        CENTER_TO_SHOOTER
            .getTranslation()
            .toTranslation2d()
            .rotateBy(drive.heading())
            .toVector()
            .times(speeds.omegaRadiansPerSecond);
    Vector<N2> hoodSpeeds = translationSpeeds.plus(rotationSpeeds);

    // get displacement from hood to from target
    Translation2d hoodTranslation = hoodPose.getTranslation();
    Translation3d displacement =
        new Translation3d(reflectedTarget.minus(hoodPose.getTranslation()));

    // run shooting alg
    Vector<N3> shotVector =
        hoodSpeeds.norm() > MINIMUM_VELOCITY
            ? algorithm.calculate(displacement, hoodSpeeds)
            : stationaryShooting.calculate(displacement, VecBuilder.fill(0, 0));

    // get values
    double rads = shotVector.get(0); // hypotenuse
    double hoodAngle = shotVector.get(1);
    double targetYaw = shotVector.get(2);
    LoggingUtils.log("/ShootingData/Distance", hoodTranslation.getDistance(reflectedTarget));

    return new ShooterParams(rads, hoodAngle, targetYaw);
  }

  /**
   * Shoots while moving
   *
   * @param target The Target
   * @param vx Forward speed for tank drive movement
   * @return Command that runs the shooting while moving
   */
  public Command shootDriving(Translation2d target, InputStream vx) {
    return Commands.waitUntil(
            () ->
                shooter.atSetpoint()
                    && shooter.setpoint() > IDLE_VELOCITY.in(RadiansPerSecond)
                    && hood.atGoal())
        .andThen(
            // TODO: do intake in parallel when done
            (runShooterSuperstructure(() -> calculateShot(target), vx)));
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
                shooter.atSetpoint()
                    && shooter.setpoint() > IDLE_VELOCITY.in(RadiansPerSecond)
                    && hood.atGoal())
        .andThen(
            // TODO: do intake in parallel when done
            runShooterSuperstructure(() -> calculateShot(target)));
  }

  /**
   * Runs the shoter at the rads, hood at the angle, and drive to point at the target angle (for
   * moving while shooting)
   *
   * @param params Shooter Params that contain flywheel rads, hood and drive angle
   * @param vx forward speed
   * @return Command to run shooter at the rads, hood at angle, and drive at target anlge
   */
  private Command runShooterSuperstructure(Supplier<ShooterParams> params, InputStream vx) {
    return Commands.parallel(
        shooter.runShooter(() -> params.get().rads),
        hood.goTo(() -> params.get().hoodAngle),
        Commands.run(() -> drive.pointAtAngle(vx.get(), params.get().driveAngle()), drive));
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
        Commands.run(() -> drive.pointAtAngle(0, params.get().driveAngle())));
  }

  /**
   * Allows us to test the shooter and hood by manually adjusting vlaues
   *
   * @return Command that runs shooter and hood for manual testing
   */
  public Command shootWithTestData() {
    return shooter
        .runShooter(() -> RADS_TEST.get())
        .alongWith(hood.goTo(() -> HOOD_DEGREES_TEST.get() * Math.PI / 180)); // in radians
  }
}
