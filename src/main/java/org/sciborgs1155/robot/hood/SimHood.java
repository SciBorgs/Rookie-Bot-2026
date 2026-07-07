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

  /* SingleJoinedArmSim is for a single pivot rotating arm
   * It also works for the variable hood as their movements are the same
   */
  private final SingleJointedArmSim simHood;

  /* Initalize the fields */
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
  public void setHoodVoltage(double voltage) {
    simHood.setInputVoltage(voltage);
    simHood.update(PERIOD.in(Seconds));
  }

  @Override
  public double getHoodPosition() {
    return simHood.getAngleRads();
  }

  @Override
  public double velocity() {
    return simHood.getVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
