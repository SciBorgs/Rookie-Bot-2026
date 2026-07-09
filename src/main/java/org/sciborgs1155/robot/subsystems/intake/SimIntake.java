package org.sciborgs1155.robot.subsystems.intake;


import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.*;


public class SimIntake implements IntakeIO{

    /**
     * init motors
     */
    private final DCMotorSim intakeSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(VELOCITY_GAIN, ACCEL_GAIN), 
            GEARBOX
        );

    @Override
    public void close() throws Exception {
        intakeSim.setInputVoltage(0);
    }

    @Override
    public void setPower(double power) {
        intakeSim.setInput(power);
    }

    @Override
    public double current() {
        return intakeSim.getCurrentDrawAmps();
    }
    
    
}
