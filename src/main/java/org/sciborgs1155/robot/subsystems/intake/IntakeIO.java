package org.sciborgs1155.robot.subsystems.intake;

public interface IntakeIO extends AutoCloseable{

    /**
     * @param power of intake 
     */
    void setPower(double power);

    /**
     * @return output current of intake
     */
    double current();

    /**
     * @return 
     */
    /**
    boolean beambreak();

    
    */
}
