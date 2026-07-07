package org.sciborgs1155.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.GEARING;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.PORT;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RealIntake implements IntakeIO{

    TalonFX motor;

    public RealIntake(){
        motor  = new TalonFX(PORT);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
        motorConfig.Feedback.SensorToMechanismRatio = GEARING;

        motor.getConfigurator().apply(motorConfig);
        motor.setVoltage(0);

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
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

}
