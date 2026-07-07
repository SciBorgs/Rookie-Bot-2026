package org.sciborgs1155.robot.shooter;

public class NoShooter implements ShooterIO {

  /* Return 0 or empty */

  @Override
  public void setFlyWheelVoltage(double voltage) {}

  @Override
  public double getFlyWheelVelocity() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
