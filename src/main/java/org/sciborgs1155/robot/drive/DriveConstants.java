package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class DriveConstants {

  /* Arbitary numbers for now */
  public static final Distance WHEEL_RADIUS = Meters.of(0.08);
  public static final double CIRCUMFERANCE = Math.PI * 2 * WHEEL_RADIUS.in(Meters);
  public static final double GEARING = 8.0;

  public static final double POSITION_FACTOR = CIRCUMFERANCE * GEARING;
  public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0;
}
