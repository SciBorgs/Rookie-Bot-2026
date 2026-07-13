package org.sciborgs1155.robot.subsystems.intake;

public interface IntakeIO extends AutoCloseable{

    /**
     * Might redo all of intake? Not sure if the system is actually going to need this many files
     *  Maybe its like the rebuilt intake? not yet sure
     */

    /**
     * @param power of intake 
     */
    void setPower(double power);

    /**
     * @return output current of intake
     */
    double current();


    
}
