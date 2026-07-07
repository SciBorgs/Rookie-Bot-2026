package org.sciborgs1155.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.*;


public class SimIntake implements IntakeIO{

    /**
     * init motors
     */
    private final SingleJointedArmSim armSim = 
        new SingleJointedArmSim(
            GEARBOX,
            GEARING,
            MOI,
            LENGTH.in(Meters),
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            START_ANGLE.in(Radians)
        );
    private final DCMotorSim intakeSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(VELOCITY_GAIN, ACCEL_GAIN), 
            GEARBOX
        );

    @Override
    public void close() throws Exception {
        armSim.setInputVoltage(0);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        intakeSim.setInputVoltage((voltage));
        intakeSim.update(PERIOD.in(Seconds));
    }

    @Override
    public void setArmVoltage(double voltage) {
        armSim.setInputVoltage(voltage);
        armSim.update(PERIOD.in(Seconds));
    }


    @Override
    public double getArmPosition() {
        return armSim.getAngleRads();
    }

    @Override
    public double getRollerVelocity() {
        return intakeSim.getAngularPositionRad();
    }

    
}
