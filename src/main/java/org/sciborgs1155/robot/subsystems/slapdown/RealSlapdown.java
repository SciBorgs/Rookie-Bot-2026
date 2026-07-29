package org.sciborgs1155.robot.subsystems.slapdown;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.subsystems.slapdown.SlapdownConstants.*;
import static org.sciborgs1155.robot.Ports.Slapdown.*;

import org.sciborgs1155.lib.FaultLogger;


public class RealSlapdown implements SlapdownIO{

    // place holder variable
    private static final boolean inverted = false;


    SparkFlex motor;
    public RealSlapdown(){
        motor = new SparkFlex(EXTENSION, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();

        config.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
        config.idleMode(IdleMode.kBrake);

        // not sure if inverted or not
        config.inverted(inverted);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(MAX_ANGLE.in(Radians));

        FaultLogger.register(motor);
    }

    @Override
    public void close() throws Exception {
        motor.close();
    }

    @Override
    public void setVoltage(double voltage) {
       motor.setVoltage(voltage);
    }

    @Override
    public double position() {
        return motor.getEncoder().getPosition();
        }
    
}
