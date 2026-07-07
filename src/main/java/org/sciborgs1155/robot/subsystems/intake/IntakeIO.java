package org.sciborgs1155.robot.subsystems.intake;

public interface IntakeIO extends AutoCloseable{

    /**
     * @param voltage of intake 
     */
    void setVoltage(double voltage);

    /**
     * @return velocity of intake
     */
    double getVelocity();
}
