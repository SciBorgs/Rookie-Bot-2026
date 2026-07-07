package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.sciborgs1155.robot.shooter.ShooterConstants.VelocityControl;

public class SimShooter implements ShooterIO {
  private final FlywheelSim simFlyWheel;

  /* Initalize the fields */
  public SimShooter() {
    simFlyWheel =
        new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                VelocityControl.V, VelocityControl.A), // physical behavior
            DCMotor.getKrakenX60(1)); // eletrical behavior
  }

  @Override
  public void setFlyWheelVoltage(double voltage) {
    simFlyWheel.setInputVoltage(voltage); // pases the voltage into the flywheel
    simFlyWheel.update(PERIOD.in(Seconds)); // updates the voltage for 0.02 seconds
  }

  @Override
  public double getFlyWheelVelocity() {
    return simFlyWheel.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
