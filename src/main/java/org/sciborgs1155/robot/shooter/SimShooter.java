package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.shooter.ShooterConstants.GEAR_RATIO;
import static org.sciborgs1155.robot.shooter.ShooterConstants.HOOD_RADIUS;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MOI;
import static org.sciborgs1155.robot.shooter.ShooterConstants.STARING_ANGLE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.robot.shooter.ShooterConstants.VelocityControl;

public class SimShooter implements ShooterIO {
  private final FlywheelSim simFlyWheel;

  /* SingleJoinedArmSim is for a single pivot rotating arm
   * It also works for the variable hood as their movements are the same
   */
  private final SingleJointedArmSim simHood;

  /* Initalize the fields */
  public SimShooter() {
    simFlyWheel =
        new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                VelocityControl.V, VelocityControl.A), // physical behavior
            DCMotor.getKrakenX60(1)); // eletrical behavior

    simHood =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            GEAR_RATIO,
            MOI,
            HOOD_RADIUS.in(Meters),
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            STARING_ANGLE.in(Radians));
  }

  @Override
  public void setFlyWheelVoltage(double voltage) {
    simFlyWheel.setInputVoltage(voltage); // pases the voltage into the flywheel
    simFlyWheel.update(PERIOD.in(Seconds)); // updates the voltage for 0.02 seconds
  }

  @Override
  public void setHoodVoltage(double voltage) {
    simHood.setInputVoltage(voltage);
    simHood.update(PERIOD.in(Seconds));
  }

  @Override
  public double getFlyWheelVelocity() {
    return simFlyWheel.getAngularVelocityRadPerSec();
  }

  @Override
  public double getHoodPosition() {
    return simHood.getAngleRads();
  }

  @Override
  public void close() throws Exception {}
}
