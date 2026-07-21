package org.sciborgs1155.robot.hood;

public class NoHood implements HoodIO {

  @Override
  public void setHoodVoltage(double voltage) {}

  @Override
  public double getHoodPosition() {
    return 0;
  }

  @Override
  public double velocity() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
