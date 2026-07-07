package org.sciborgs1155.robot.subsystems.intake;

public class NoIntake implements IntakeIO{

    
    @Override
    public void close() throws Exception {}

    @Override
    public void setRollerVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRollerVoltage'");
    }

    @Override
    public void setArmVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setArmVoltage'");
    }
    @Override
    public double getArmPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getArmPosition'");
    }

    @Override
    public double getRollerVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRollerVelocity'");
    }

    
}
