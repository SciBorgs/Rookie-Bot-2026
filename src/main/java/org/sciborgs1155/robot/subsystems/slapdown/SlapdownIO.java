package org.sciborgs1155.robot.subsystems.slapdown;

public interface SlapdownIO extends AutoCloseable{
        /**
     * @param voltage of the arm extending
     */
    void setVoltage(double voltage);

    /**
     * @return position of arm when extended
     */
    double position();

}
