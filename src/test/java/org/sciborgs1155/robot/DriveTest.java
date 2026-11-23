package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_SPEED;
import static org.sciborgs1155.robot.drive.DriveConstants.VELOCITY_TOLERANCE;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.drive.Drive;

public class DriveTest {

  private Drive drive;

  @BeforeEach
  public void setup() {
    setupTests();
    drive = Drive.create();
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(drive);
  }

  @RepeatedTest(5)
  public void testDrive() {
    double val = Math.random();
    double setpoint = val * MAX_SPEED.in(MetersPerSecond);
    run(drive.run(() -> drive.setChassisSpeeds(new ChassisSpeeds(setpoint, 0, 0))));
    fastForward(Seconds.of(5));
    assertEquals(
        setpoint,
        drive.robotRelativeChassisSpeeds().vxMetersPerSecond,
        VELOCITY_TOLERANCE.in(MetersPerSecond));
  }
}
