package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
    fastForward(Seconds.of(3));
    double val = Math.random();
    CommandScheduler.getInstance()
        .schedule(
            hood.goTo(() -> val * MAX_ANGLE.in(Radians)).withDeadline(Commands.waitSeconds(4)));
    System.out.println(val * MAX_ANGLE.in(Radians));
    fastForward(Seconds.of(3));
    assertEquals(val * MAX_ANGLE.in(Radians), hood.angle(), POSITION_TOLERANCE.in(Radians));
    System.out.println(hood.angle());
  }
}
