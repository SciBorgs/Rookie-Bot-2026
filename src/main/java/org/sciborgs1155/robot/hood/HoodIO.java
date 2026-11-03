package org.sciborgs1155.robot.hood;

public interface HoodIO extends AutoCloseable {
  /**
   * sets voltage of variable hood
   *
   * @param voltage The voltage
   */
  void setVoltage(double voltage);

  /**
   * Position of hood in radians
   *
   * @return the position of hood in radians
   */
  double getPosition();

  /**
   * Current hood velocity
   *
   * @return velocity in radians per sec
   */
  double velocity();
}
