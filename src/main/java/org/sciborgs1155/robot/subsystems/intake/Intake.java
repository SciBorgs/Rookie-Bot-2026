package org.sciborgs1155.robot.subsystems.intake;

import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.INTAKE_POWER;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements AutoCloseable{

    private final IntakeIO hardware;
    /**
     * 
     * @param hardware is the hardware intake will be opearted on
     */
    public Intake(IntakeIO hardware){
        this.hardware = hardware;

        setDefaultCommand(stop());
    }


    /**
    * @return a real intake if the intake is real and a simmed intake if it is not in order to
    *     simulate the arm
    */
    public static Intake create() {
        return new Intake(Robot.isReal() ? new RealIntake() : new SimIntake());
    }

    /**
     * 
     * @return a non-implemented roller
     */
    public static Intake none(){
        return new Intake(new NoIntake());
    }


    /**
     * 
     * @return start rollers in order to intake fuel
     */
    public Command spin(double power) {
        return run(() -> hardware.setPower(power));
    }
    
    /**
     * 
     * @return make the motors spin to intake balls
     */
    public Command intake() {
        return spin(INTAKE_POWER);
    }
    /**
     * 
     * @return make the motors spin to outtake balls
     */
    public Command outake() {
        return spin(-INTAKE_POWER);
    }

    /**
     * 
     * @return stop the motors
     */
    public Command stop(){
        return spin(0);
    }
    @Override
    public void close() throws Exception {
        hardware.close();
    }
    
}
