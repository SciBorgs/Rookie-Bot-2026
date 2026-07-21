package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class HoodConstants {
  public static final Current SUPPLY_LIMIT = Amps.of(30);
  public static final Current STATOR_LIMIT = Amps.of(30);
  public static final double Gear_RATIO = 10.0; // FOR NOW
  public static final Angle MIN_ANGLE = Degrees.of(15);
  public static final Angle MAX_ANGLE = Degrees.of(53);
  public static final Mass MASS = Pounds.of(1.307);
  public static final Angle STARTING_ANGLE = MIN_ANGLE;
  public static final Distance HOOD_RADIUS = Inches.of(9.29);
  public static final double MOI = 0.0045821517; // kg*m^2
}
