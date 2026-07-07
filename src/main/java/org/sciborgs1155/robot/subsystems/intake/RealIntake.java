package org.sciborgs1155.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radian;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.EXTENSION;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.GEARING;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.GEARING_INTAKE;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.subsystems.intake.IntakeConstants.ROLLER;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RealIntake implements IntakeIO{

    TalonFX armMotor;
    TalonFX intakeMotor;

    public RealIntake(){
        armMotor = new TalonFX(EXTENSION);
        intakeMotor = new TalonFX(ROLLER);

        TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();

        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armMotorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
        armMotorConfig.Feedback.SensorToMechanismRatio = GEARING;

        TalonFXConfiguration intakeMtorConfig = new TalonFXConfiguration();

        intakeMtorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeMtorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
        intakeMtorConfig.Feedback.SensorToMechanismRatio = GEARING_INTAKE;

        armMotor.getConfigurator().apply(armMotorConfig);
        armMotor.setPosition(MAX_ANGLE);

        intakeMotor.getConfigurator().apply(intakeMtorConfig);
        intakeMotor.setVoltage(0);

        TalonUtils.addMotor(armMotor);
        FaultLogger.register(armMotor);

        TalonUtils.addMotor(intakeMotor);
        FaultLogger.register(intakeMotor);
    }
    
    @Override
    public void close() throws Exception {
        intakeMotor.close();
        armMotor.close();
    }

    @Override
    public void setRollerVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    @Override
    public void setArmVoltage(double voltage) {
        armMotor.setVoltage(voltage);
    }

    @Override
    public double getArmPosition() {
        return armMotor.getPosition().getValue().in(Radian);
    }

    @Override
    public double getRollerVelocity() {
        return intakeMotor.getVelocity().getValueAsDouble();
    }

}
