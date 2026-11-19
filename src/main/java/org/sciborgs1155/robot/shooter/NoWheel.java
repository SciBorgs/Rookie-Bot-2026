package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class NoWheel implements WheelIO {

  /* Return 0 or empty */

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(0);
  }

  @Override
  public void close() throws Exception {}
}
