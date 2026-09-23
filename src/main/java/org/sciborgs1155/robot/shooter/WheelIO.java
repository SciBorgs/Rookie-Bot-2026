package org.sciborgs1155.robot.shooter;

public interface WheelIO extends AutoCloseable {

  /*
   * sets voltage of flywheel
   * @param voltage The voltage
   */

  void setVoltage(double voltage);

  /*
   * velocity of the flywheel (in radians per sec)
   * @return The velocity of the flywheel, in radians per sec
   *
   */
  double getVelocity();
}
