package org.sciborgs1155.robot.hood;

public interface HoodIO extends AutoCloseable {
  /*
   * sets voltage of variable hood
   * @param voltage The voltage
   */
  void setHoodVoltage(double voltage);

  /*
   * Posotion of hood in radians
   * @returns the position of hood in radians
   */
  double getHoodPosition();

  /*
   * Current hood velocity
   * @returns velocity in radians per sec
   */
  double velocity();
}
