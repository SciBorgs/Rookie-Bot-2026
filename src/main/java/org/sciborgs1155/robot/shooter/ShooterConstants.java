package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;

public class ShooterConstants {
  public static final double GEAR_RATIO = 10.0;
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

  /* Below are control constants for flyhweel and hood
   * I'm using the Rebuilt values for now
   * Will change later
   */

  /* For Flywheel */
  public static class VelocityControl {
    public static final double P = 0.03;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = 0.016981;
    public static final double A = 0.0021296;
  }

  /* For hood */
  public static class AngularControl {
    public static final double P = 0.03;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double S = 0.0;
    public static final double V = 0.016981;
    public static final double A = 0.0021296;
  }
}
