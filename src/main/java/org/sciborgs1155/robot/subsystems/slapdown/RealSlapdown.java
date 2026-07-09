package org.sciborgs1155.robot.subsystems.slapdown;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.subsystems.slapdown.SlapdownConstants.*;
import static org.sciborgs1155.robot.Ports.Slapdown.*;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;


public class RealSlapdown implements SlapdownIO{

    TalonFX motor;
    public RealSlapdown(){
        motor = new TalonFX(EXTENSION);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
        motorConfig.Feedback.SensorToMechanismRatio = GEARING;

        motor.getConfigurator().apply(motorConfig);
        motor.setPosition(MAX_ANGLE);

        TalonUtils.addMotor(motor);
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
        return motor.getPosition().getValue().in(Radians);
        }
    
}
