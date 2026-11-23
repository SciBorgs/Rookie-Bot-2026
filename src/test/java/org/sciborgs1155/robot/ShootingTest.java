package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.commands.Shooting;
import org.sciborgs1155.robot.commands.Shooting.ShooterParams;
import org.sciborgs1155.robot.commands.shooting.BasketballVisualizer;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.shooter.ShooterConstants;
import static edu.wpi.first.units.Units.Meters;
import org.sciborgs1155.robot.shooter.ShooterConstants;

public class ShootingTest {
  Shooting shooting;
  Drive drive;

  @BeforeEach
  public void setup() {
    setupTests();
    drive = new Drive();

    shooting =
        new Shooting(
            Shooter.create(),
            Hood.create(),
            new BasketballVisualizer(
                () -> new double[] {0, 0, 0},
                () -> new double[] {0, 0, 0},
                () -> new double[] {0, 0, 0},
                () -> 0),
            drive);
  }

  @AfterEach
  public void cleanup() throws Exception {
    if (drive != null) {
      reset(drive);
    } else {
      reset();
    }
  }

  /**
   * Simulates a stationary shot at HUB_TARGET using the full projectile physics and asserts the
   * ball scores. This validates that the lookup table and algorithm produce a shot that actually
   * makes it in.
   */
  @Test
  public void stationaryShot() {
    Translation2d HUB_TARGET = Shooting.HUB_TARGET;
    ShooterParams params = shooting.calculateShot(HUB_TARGET);
    Pose3d robotPose = new Pose3d(drive.pose());

    double[] shotVel =
    BasketballVisualizer.shotVelocity(
        params.rads() * ShooterConstants.RADIUS.in(Meters),
        Math.PI / 2 - params.hoodAngle(),
        params.driveAngle() - robotPose.getRotation().getZ(),
        robotPose
      );

    BasketballVisualizer visualizer =
        new BasketballVisualizer(() -> shotVel, () -> robotPose, ChassisSpeeds::new);
    visualizer.generateTrajectory();

    assertTrue(visualizer.willScore());
  }
}
