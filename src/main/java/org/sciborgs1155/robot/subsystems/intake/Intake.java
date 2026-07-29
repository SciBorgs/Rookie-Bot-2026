package org.sciborgs1155.robot.subsystems.intake;

import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.INTAKE_POWER;
import static org.sciborgs1155.robot.Ports.Intake.*;
import static edu.wpi.first.units.Units.Amps;

import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements AutoCloseable{

    // place holder variable
    private static final boolean inverted = false;

    // hardware init
    private final SimpleMotor hardware;

    public Intake(SimpleMotor hardware){
        this.hardware = hardware;
        setDefaultCommand(stop());
    }
    /**
     * 
     * @return real intake if intake is real, sim if no arm sim
     */
    public static Intake create(){
        return Robot.isReal() ? new Intake(realMotor()) : none();
    }

    /**
     * 
     * @return simple motor for rollers
     */
    private static SimpleMotor realMotor(){
        final SparkFlex motor = new SparkFlex(ROLLERS, SparkLowLevel.MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
        config.idleMode(IdleMode.kBrake);

        // not sure if inverted or not
        config.inverted(inverted);

        return SimpleMotor.spark(motor, config);
    }


    /**
     * @return create new intake without hardware
     */
    public static Intake none(){
        return new Intake(SimpleMotor.none());
    }


    /**
     * 
     * @param power
     * @return start rolling rollers at power
     */
    public Command spin(double power){
        return run(() -> hardware.set(power));
    }

    /**
     * @return make intake motors spin for outake
     */
    public Command intake(){
        return spin(INTAKE_POWER);
    }

    /**
     * 
     * @return make intake motors spin for intake
     */
    public Command outtake(){
        return spin(-INTAKE_POWER);
    }

    /**
     * @return stop motors
     */
    public Command stop(){
        return spin(0);
    }

    /**
     * close hardware
     */
    @Override
    public void close() throws Exception {
        hardware.close();
    }
    
    
}
