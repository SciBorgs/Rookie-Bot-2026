package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public class DriveConstants {

  /* Arbitary numbers for now */
  public static final Distance WHEEL_RADIUS = Meters.of(0.08);
  public static final double CIRCUMFERANCE = Math.PI * 2 * WHEEL_RADIUS.in(Meters);
  public static final double GEARING = 8.0;

  public static final double POSITION_FACTOR = CIRCUMFERANCE * GEARING;
  public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0;
  
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(2); //meters per sec 

    public static final Distance TRACK_WIDTH = Meters.of(0.7112);
  public static final double MOI = 7.5;
  public static final Mass DRIVE_MASS = Kilograms.of(60.0); //kg
  public static final Matrix<N7, N1> STD_DEVS = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0);
  
  //for auto
  public static final Current STATOR_LIMIT = Amps.of(80); // 120A max slip current

  // Coefficient of friction between the drive wheel and the carpet.
  public static final double WHEEL_COF = 1.0;

  public static final Translation2d[] wheelInfo = {new Translation2d(0.0, 1.0), new Translation2d(0.0, -1)}; //fake vales
  

  public static final class FF {
    public static final double kS = 1;
    public static final double kV = 3;
  }

  public static final class PID {
    public static final double kP = 8.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
}
