package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.shooter.ShooterConstants.AngularControl.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VelocityControl.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.sciborgs1155.robot.shooter.ShooterConstants.VelocityControl;

public class SimShooter implements ShooterIO {
  private final FlywheelSim flywheel;

  public SimShooter() {
    flywheel =
        new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                VelocityControl.V, VelocityControl.A), // physical behavior
            DCMotor.getKrakenX60(1)); // eletrical behavior
  }

  @Override
  public void setFlyWheelVoltage(double voltage) {
    flywheel.setInputVoltage(voltage); // pases the voltage into the flywheel
    flywheel.update(PERIOD.in(Seconds)); // updates the voltage for 0.02 seconds
  }

  @Override
  public double getFlyWheelVelocity() {
    return flywheel.getAngularVelocityRadPerSec();
  }
}
