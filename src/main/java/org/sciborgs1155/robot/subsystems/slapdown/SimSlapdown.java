
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.subsystems.slapdown.SlapdownConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;


public class SimSlapdown implements SlapdownIO{

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
    
    @Override
    public void close() throws Exception {
        armSim.setInputVoltage(0);
    }
    @Override
    public void setVoltage(double voltage) {
        armSim.setInputVoltage(voltage);
        armSim.update(PERIOD.in(Seconds));
    }


    @Override
    public double position() {
        return armSim.getAngleRads();
    }


    
}
