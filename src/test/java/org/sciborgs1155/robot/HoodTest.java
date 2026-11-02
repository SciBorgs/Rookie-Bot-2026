package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.hood.Hood;

public class HoodTest {

  private Hood hood;

  @BeforeEach
  public void initalize() {
    setupTests();
    hood = Hood.create();
  }

  /** resets the sim hood */
  @AfterEach
  public void destroy() throws Exception {
    reset(hood);
  }

  /** Tests whether changing the position of the hood tracks correctly. */
  @RepeatedTest(5)
  public void orientation() {
    double val = Math.random();
    CommandScheduler.getInstance().schedule(hood.goTo(() -> val * MAX_ANGLE.in(Radians)));
    fastForward(Seconds.of(3));
    assertEquals(hood.angle(), val * MAX_ANGLE.in(Radians), POSITION_TOLERANCE.in(Radians));
  }
}
