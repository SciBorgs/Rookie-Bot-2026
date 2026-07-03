package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;

public class ShooterConstants {
  public static final double GEAR_RATIO = 10.0;
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);
}
