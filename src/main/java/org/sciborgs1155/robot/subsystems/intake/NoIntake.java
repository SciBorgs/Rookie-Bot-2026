package org.sciborgs1155.robot.subsystems.intake;

public class NoIntake implements IntakeIO{

    
    @Override
    public void close() throws Exception {}

    @Override
    public void setPower(double power) {}

    @Override
    public double current() {
        return 0;
    }


    
}
