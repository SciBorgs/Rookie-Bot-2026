package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {

  /*
   *ALL OF THESE VALUES ARE PLACE HOLDERS
   *WILL CHANGE AFTER CAD AND TESTING IS DONE
   */

  public static final double GEAR_RATIO =
      //10.0; // 10: 1 ratio (every 10 motor spins = 1 mechanism spin)
      48.0/36.0;

  //36/48

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);
  public static final double MOI = 0.0410396299; // Moment of Intertia for every part spinning with shooter

  //all parts driven by shooter
  //10 wheels, each 0.057383 MOI 0.57383
  //TOTAL unconverted 140.23947


  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(1);
  public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(5);
  public static final AngularVelocity MAX_VELOCITY = RPM.of(7230);  
  public static final double MAX_VOLTAGE = 12.0; // from rebuilt


  //floor center of robot: -13.502in,-17.654in,0 (Z-axis is vertical when viewed at front)
  //shooter center: 10.090483in,11.049in,4.392in
  public static final Transform3d CENTER_TO_SHOOTER =
      new Transform3d(Inches.of(23.592483), Inches.of(28.703), Inches.of(4.392), new Rotation3d()); 

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
