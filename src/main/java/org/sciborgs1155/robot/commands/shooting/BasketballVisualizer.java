package org.sciborgs1155.robot.commands.shooting;

import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.sciborgs1155.robot.FieldConstants.Hub;

/**
 * A class that manages the creation, simulation, and logging of simulated Basketball projectiles.
 *
 * @see Basketball
 */
public class BasketballVisualizer extends ProjectileVisualizer {
  /**
   * A class that manages the creation, simulation, and logging of simulated Basketball projectiles.
   *
   * @param launchVelocity a supplier that provides the velocity of the Basketball at launch time
   * @param robotPose a supplier that provides the pose of the robot at launch time
   * @param robotVelocity a supplier that provides the velocity of the robot at launch time
   */
  public BasketballVisualizer(
      Supplier<double[]> launchVelocity,
      Supplier<Pose3d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity) {
    super(
        () -> launchTranslation(robotPose.get()),
        () -> launchVelocity(launchVelocity.get(), robotPose.get(), robotVelocity.get()),
        () -> launchRotation(launchVelocity.get(), robotPose.get()),
        () -> launchRotationalVelocity());
  }

  /**
   * A class that manages the creation, simulation, and logging of simulated Basketball projectiles.
   *
   * @param launchTranslation a supplier that provides the translation of the Basketball at launch time
   * @param launchVelocity a supplier that provides the velocity of the Basketball at launch time
   * @param launchRotation a supplier that provides the rotation of the Basketball at launch time
   * @param launchRotationalVelocity a supplier that provides the rotational velocity of the Basketball at
   *     launch time
   */
  public BasketballVisualizer(
      Supplier<double[]> launchTranslation,
      Supplier<double[]> launchVelocity,
      Supplier<double[]> launchRotation,
      DoubleSupplier launchRotationalVelocity) {
    super(launchTranslation, launchVelocity, launchRotation, launchRotationalVelocity);
  }

  @Override
  protected Projectile createProjectile(
      double resolution,
      boolean weightEnabled,
      boolean dragEnabled,
      boolean torqueEnabled,
      boolean liftEnabled) {
    return new Basketball().config(resolution, weightEnabled, dragEnabled, torqueEnabled, liftEnabled);
  }

  protected static double[] launchTranslation(Pose3d robotPose) {
    double[] robotTranslation = {robotPose.getX(), robotPose.getY(), robotPose.getZ()};
    return Projectile.add3(robotToShooter(robotPose), robotTranslation);
  }

  protected static double[] launchVelocity(
      double[] shotVelocity, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    return Projectile.add3(shotVelocity, shooterVelocity(shotVelocity, robotPose, robotVelocity));
  }

  protected static double[] launchRotation(double[] shotVelocity, Pose3d robotPose) {
    double[] axis = Projectile.rotateAroundZ(shotVelocity, Math.PI / 2.0);
    return Projectile.scale4(
        new double[] {0, axis[Basketball.X], axis[Basketball.Y], axis[Basketball.Z]},
        1 / Projectile.norm3(shotVelocity));
  }

  protected static double launchRotationalVelocity() {
    return 0.5; // TODO: UPDATE.
  }

  /**
   * Converts shooter properties to a shot velocity vector (X, Y, and Z) which is compatible with
   * visualizers.
   *
   * @param speed the launch speed of the Basketball.
   * @param pitch the pitch of the shooter.
   * @param yaw the yaw of the shooter.
   * @param robotPose the pose of the drivetrain.
   * @return A double[] that can be passed into the constructor of a visualizer.
   */
  public static double[] shotVelocity(double speed, double pitch, double yaw, Pose3d robotPose) {
    return Projectile.scale3(
        Basketball.rotateAroundZ(
            Projectile.toDirectionVector(pitch, yaw), robotPose.getRotation().getZ()),
        speed);
  }

  /**
   * Converts a shooting algorithm output to a shot velocity vector (X, Y, and Z) which is
   * compatible with visualizers.
   *
   * @param shootingAlgorithm the shooting algorithm used to calculate the shot velocity.
   * @param robotPose the pose of the drivetrain.
   * @param robotVelocity the velocity of the drivetrain.
   * @return A double[] that can be passed into the constructor of a visualizer.
   */
  public static double[] shotVelocity(
      ShootingAlgorithm shootingAlgorithm, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    return shootingAlgorithm
        .calculate(
            robotPose.getTranslation(),
            VecBuilder.fill(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond))
        .getData();
  }

  protected static double[] robotToShooter(Pose3d robotPose) {
    return Projectile.rotateAroundZ(
        Projectile.fromTranslation(CENTER_TO_SHOOTER.getTranslation()),
        robotPose.getRotation().getZ());
  }

  protected static double[] shooterVelocity(
      double[] shotVelocity, Pose3d robotPose, ChassisSpeeds robotVelocity) {
    double tangentialSpeed =
        robotVelocity.omegaRadiansPerSecond
            * Projectile.norm3(robotToShooter(robotPose));
    double tangentialDirection = robotPose.getRotation().getZ() + Math.PI / 2.0;

    double xVelocity =
        robotVelocity.vxMetersPerSecond + tangentialSpeed * Math.cos(tangentialDirection);
    double yVelocity =
        robotVelocity.vyMetersPerSecond + tangentialSpeed * Math.sin(tangentialDirection);

    return new double[] {xVelocity, yVelocity, 0};
  }

  /** Models the launch physics of a Basketball projectile. */
  public static class Basketball extends Projectile {
    /** Mass of the Basketball projectile in kilograms. */
    protected static final double Basketball_MASS = 0.61; // Original 0.225

    /** Radius of the Basketball projectile in meters. */
    protected static final double Basketball_RADIUS = 0.118; // Original 0.075

    protected static final double SCORE_TOLERANCE = 0;
    protected static final double GRAVITY = -9.80665;
    protected static final double AIR_DENSITY = 1.225;
    protected static final double AIR_VISCOSITY = 15.24 * Math.pow(10, -6);

    /** Multiplied by velocity squared to compute drag force. */
    private static final double DRAG_CONSTANT =
        0.5 * 0.47 * AIR_DENSITY * Math.PI * Basketball_RADIUS * Basketball_RADIUS;

    /** Multiplied by velocity * angular speed to compute lift force. */
    private static final double LIFT_CONSTANT =
        4 / 3 * 4 * Math.PI * Math.PI * Basketball_RADIUS * Basketball_RADIUS * Basketball_RADIUS * AIR_DENSITY;

    /** Multiplied by angular speed to compute torque. */
    private static final double TORQUE_CONSTANT =
        -8 * Math.PI * AIR_VISCOSITY * Math.pow(Basketball_RADIUS, 3);

    @Override
    protected double[] weight() {
      // SOURCE: https://spaceplace.nasa.gov/what-is-gravity/en/
      return new double[] {0, 0, GRAVITY};
    }

    @Override
    protected double[] drag() {
      // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-of-a-sphere/
      return new double[] {
        velocity[X] * velocity[X] * DRAG_CONSTANT / Basketball_MASS,
        velocity[Y] * velocity[Y] * DRAG_CONSTANT / Basketball_MASS,
        velocity[Z] * velocity[Z] * DRAG_CONSTANT / Basketball_MASS
      };
    }

    @Override
    protected double torque() {
      // https://physics.wooster.edu/wp-content/uploads/2021/08/Junior-IS-Thesis-Web_1998_Grugel.pdf
      return rotationalVelocity * TORQUE_CONSTANT / Basketball_MASS;
    }

    @Override
    protected double[] lift() {
      // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-lift-of-a-spinning-ball/
      return new double[] {0, 0, LIFT_CONSTANT * norm3(velocity) * rotationalVelocity / Basketball_MASS};
    }

    @Override
    protected boolean willScore() {
      double hub1XDisplacement = translation[X] - Hub.TOP_CENTER_POINT.getX();
      double hub1YDisplacement = translation[Y] - Hub.TOP_CENTER_POINT.getY();

      double hub2XDisplacement = translation[X] - Hub.OPP_TOP_CENTER_POINT.getX();
      double hub2YDisplacement = translation[Y] - Hub.OPP_TOP_CENTER_POINT.getY();

      double hub1Distance = Math.hypot(hub1XDisplacement, hub1YDisplacement);
      double hub2Distance = Math.hypot(hub2XDisplacement, hub2YDisplacement);

      double planarDistance = Math.min(hub1Distance, hub2Distance);
      double verticalDisplacement = Hub.HEIGHT - translation[Z];
      double scoreRadius = SCORE_TOLERANCE + Basketball_RADIUS + Hub.WIDTH / 2;

      return verticalDisplacement < 0
          && verticalDisplacement > -Basketball_RADIUS
          && planarDistance <= scoreRadius
          && velocity[Z] < 0;
    }

    @Override
    protected boolean willMiss() {
      double hub1XDisplacement = translation[X] - Hub.TOP_CENTER_POINT.getX();
      double hub1YDisplacement = translation[Y] - Hub.TOP_CENTER_POINT.getY();

      double hub2XDisplacement = translation[X] - Hub.OPP_TOP_CENTER_POINT.getX();
      double hub2YDisplacement = translation[Y] - Hub.OPP_TOP_CENTER_POINT.getY();

      double hub1Distance = Math.hypot(hub1XDisplacement, hub1YDisplacement);
      double hub2Distance = Math.hypot(hub2XDisplacement, hub2YDisplacement);

      double planarDistance = Math.min(hub1Distance, hub2Distance);
      double verticalDisplacement = Hub.HEIGHT - translation[Z];
      double scoreRadius = SCORE_TOLERANCE + Basketball_RADIUS + Hub.WIDTH / 2;

      return (verticalDisplacement > -Basketball_RADIUS
              && planarDistance > scoreRadius
              && velocity[Z] < 0)
          || translation[Z] < Basketball_RADIUS;
    }
  }
}
