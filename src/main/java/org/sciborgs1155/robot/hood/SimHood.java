package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.hood.HoodConstants.Gear_RATIO;
import static org.sciborgs1155.robot.hood.HoodConstants.HOOD_RADIUS;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MOI;
import static org.sciborgs1155.robot.hood.HoodConstants.STARTING_ANGLE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MIN_ANGLE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimHood implements HoodIO {

  /** Using SingleJoinedArmSim as same movements (single pivot point) */
  private final SingleJointedArmSim simHood;

  /* Initalize the field*/
  public SimHood() {
    simHood =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Gear_RATIO,
            MOI,
            HOOD_RADIUS.in(Meters),
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            STARTING_ANGLE.in(Radians));
  }

  @Override
  public void setVoltage(double voltage) {
    simHood.setInputVoltage(voltage);
    simHood.update(PERIOD.in(Seconds));
  }

  @Override
  public double getPosition() {
    return simHood.getAngleRads();
  }

  @Override
  public double velocity() {
    return simHood.getVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
