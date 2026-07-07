package org.sciborgs1155.robot.subsystems.slapdown;

public class NoSlapdown implements SlapdownIO{

    @Override
    public void close() throws Exception {}

    @Override
    public void setVoltage(double voltage) {}

    @Override
    public double position() {
        return 0;
    }
    
}
