package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {

  /*
   *ALL OF THESE VALUES ARE PLACE HOLDERS
   *WILL CHANGE AFTER CAD AND TESTING IS DONE
   */

  public static final double GEAR_RATIO =
      10.0; // 10: 1 ratio (every 10 motor spins = 1 mechanism spin)
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);
  public static final double MOI = 0.01; // Moment of Intertia

  // copied from rebuilt
  public static final Distance HOOD_RADIUS = Inches.of(9.29);
  public static final Angle MIN_ANGLE = Degrees.of(15);
  public static final Angle MAX_ANGLE = Degrees.of(15);
  public static final Angle STARING_ANGLE = MIN_ANGLE;
  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(1);
  public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(5);
  public static final AngularVelocity MAX_VELOCITY = RPM.of(7230);
  public static final double MAX_VOLTAGE = 12.0; // from rebuilt



  /* For Flywheel */
  public static class VelocityControl {
    public static final double P = 0.03;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = 0.016981;
    public static final double A = 0.0021296;
  }
  
}
