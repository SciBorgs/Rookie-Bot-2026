package org.sciborgs1155.robot.shooter;

public interface ShooterIO extends AutoCloseable {

  /*
   * sets voltage of flywheel
   * @param voltage The voltage
   */

  void setFlyWheelVoltage(double voltage);

  /*
   * sets voltage of variable hood
   * @param voltage The voltage
   */
  void setHoodVoltage(double voltage);

  /*
   * velocity of the flywheel (in radians per sec)
   * @return The velocity of the flywheel, in radians per sec
   *
   */
  double getFlyWheelVelocity();

  /*
   * Posotion of hood in radians
   * @returns the position of hood in radians
   */
  double getHoodPosition();
}
