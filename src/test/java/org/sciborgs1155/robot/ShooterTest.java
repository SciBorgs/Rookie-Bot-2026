package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
// import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.shooter.Shooter;
import org.sciborgs1155.robot.shooter.ShooterConstants;

public class ShooterTest {
  private Shooter shooter;

  @BeforeEach
  public void setup() {
    setupTests();
    shooter = Shooter.create();
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(shooter);
  }

  @RepeatedTest(5)
  public void randVelocity() throws Exception {
    //fastForward(Seconds.of(3));
    double setpoint = Math.random() * ShooterConstants.MAX_VELOCITY.in(RadiansPerSecond);
    CommandScheduler.getInstance().schedule(shooter.runShooter(() -> setpoint));
    fastForward(Seconds.of(3));
    assertEquals(
        shooter.getVelocity().in(RadiansPerSecond),
        setpoint,
        VELOCITY_TOLERANCE.in(RadiansPerSecond));
        //1000);
  }
}
