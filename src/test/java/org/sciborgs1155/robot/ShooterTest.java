package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
// import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.shooter.Shooter;

public class ShooterTest {
  private Shooter shooter;

  @RepeatedTest(5)
  public void randVelocity() throws Exception {
    double val = Math.random();
    double setpoint = val * MAX_VELOCITY.in(RadiansPerSecond);
    CommandScheduler.getInstance()
        .schedule(shooter.runShooter(setpoint));
    fastForward(Seconds.of(3));
    assertEquals(
        shooter.setpoint(),
        shooter.getVelocity().in(RadiansPerSecond),
        VELOCITY_TOLERANCE.in(RadiansPerSecond));
  }
}
