package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.drive.Drive;

public class DriveTest {

  @BeforeEach
  public void setup() {
    setupTests();
    Drive drive = Drive.create();
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(drive);
  }

  @RepeatedTest(5)
  public void testDrive() {
    double val = Math.random();
    double setpoint = val * MAX_ANGLE.in(Radians);
    assertEquals(
        setpoint, drive.getPosition().in(Radians), DRIVE_TOLERANCE.in(Radians).in(Radians));
  }
}
