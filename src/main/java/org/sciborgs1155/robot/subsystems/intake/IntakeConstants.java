package org.sciborgs1155.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;


public class IntakeConstants {
    // COPIED FROM REBUILT 2026
  public static final Current CURRENT_LIMIT = Amps.of(30);

  public static final double INTAKE_POWER = 0.5;
  public static final double GEARING = 2;


  // NOT COPIED... PLACEHOLDER VARIABLES
  public static final DCMotor GEARBOX = null;

  public static final double ACCEL_GAIN = 0;
  public static final double VELOCITY_GAIN = 0;
}
