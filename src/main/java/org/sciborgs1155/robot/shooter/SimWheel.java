package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static org.sciborgs1155.robot.shooter.ShooterConstants.GEAR_RATIO;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MOI;

public class SimWheel implements WheelIO {
  private final FlywheelSim simFlyWheel;
  private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), MOI, GEAR_RATIO);

  /* Initalize the fields */
  public SimWheel() {
    simFlyWheel = new FlywheelSim(plant, DCMotor.getNeoVortex(2));
     
  }

  @Override
  public void setVoltage(double voltage) {
    simFlyWheel.setInputVoltage(voltage); // pases the voltage into the flywheel
    simFlyWheel.update(PERIOD.in(Seconds)); // updates the voltage for 0.02 seconds
  }

  @Override
  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(simFlyWheel.getAngularVelocityRadPerSec());
  }

  @Override
  public void close() throws Exception {}
}
