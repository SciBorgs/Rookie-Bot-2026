package org.sciborgs1155.robot.shooter;

public class NoWheel implements WheelIO {

  /* Return 0 or empty */

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
