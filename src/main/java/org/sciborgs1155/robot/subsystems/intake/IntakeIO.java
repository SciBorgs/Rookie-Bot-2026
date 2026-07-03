package org.sciborgs1155.robot.subsystems.intake;

public interface IntakeIO extends AutoCloseable{
    /**
     * uh this is subject to change
     * maybe might have intake as separate subsystem
     * like on the 2026 bot?
     */
    /**
     * @param voltage of the arm extending
     */
    void setArmVoltage(double voltage);

    /**
     * @param voltage of roller 
     */
    void setRollerVoltage(double voltage);

    /**
     * @return position of arm when extended
     */
    double arnPosition();

}
