package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.Shooting.MINIMUM_VELOCITY;
import static org.sciborgs1155.robot.FieldConstants.allianceReflect;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;
import static org.sciborgs1155.robot.shooter.ShooterConstants.IDLE_VELOCITY;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.commands.shooting.FuelVisualizer;
import org.sciborgs1155.robot.commands.shooting.MovingShot;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting {

  public static final DoubleEntry LATENCY_TIME = Tuning.entry("/ShootingData/Latency Time", 0.1);
  private final MovingShot algorithm = new MovingShot();
  private final Shooter shooter;
  private final Hood hood;
  private final FuelVisualizer fuelVisualizer;
  private final Drive drive;

  private Translation2d lastTarget = new Translation2d();

  public Shooting(Shooter shooter, Hood hood, FuelVisualizer fuelVisualizer, Drive drive) {
    this.shooter = shooter;
    this.hood = hood;
    this.fuelVisualizer = fuelVisualizer;
    this.drive = drive;
  }

  /** rads for fly wheel hood angle drive angle to orientate the tank drive */
  public record ShooterParams(double rads, double hoodAngle, double driveAngle) {}

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
            : null; // TODO: null

    double rads = shotVector.get(0); // hypotenuse
    double hoodAngle = shotVector.get(1);
    double targetYaw = shotVector.get(2);
    LoggingUtils.log("/ShootingData/Distance", hoodTranslation.getDistance(reflectedTarget));

    return new ShooterParams(rads, hoodAngle, targetYaw);
  }

    public Command shootDriving(Translation2d target, InputStream vx, InputStream vy, InputStream omega) {
        return Commands.waitUntil(
            () -> 
            shooter.atSetpoint()
            && shooter.setpoint() > IDLE_VELOCITY.in(RadiansPerSecond)
            && hood.atGoal())
        .andThen(
            // do other mechenisms in parallel when done
            Commands.run(
                () -> {fuelVisualizer != null) fuelVisualizer.launchProjectile();
            ).deadlineFor()

        )
    }

    private Command runShooterSuperstructure(Supplier<ShooterParams> params) {
    return Commands.parallel(
        shooter.runShooter(() -> params.get().rads),
        hood.goTo(() -> params.get().hoodAngle),
        drive.goToYaw(() -> Rotation2d.fromRadians(params.get().turretAngle)));
  }


}
