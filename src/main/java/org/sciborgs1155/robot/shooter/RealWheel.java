package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Shooter.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealWheel implements WheelIO {

  private final TalonFX wheelMotor;

  /* Sets motor configurations */
  public RealWheel() {

    wheelMotor = new TalonFX(WHEEL_MOTOR);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // stops the motor
    configs.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // counterclockwise is postive
    configs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT.in(Amps);
    configs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);

    wheelMotor.getConfigurator().apply(configs);

    /* Checks the motors */
    FaultLogger.register(wheelMotor);

    /* adds motors to a list of all global motors */
    TalonUtils.addMotor(wheelMotor);
  }

  @Override
  public void setFlyWheelVoltage(double voltage) {
    wheelMotor.setVoltage(voltage);
  }

  @Override
  public double getFlyWheelVelocity() {
    return wheelMotor.getVelocity().getValueAsDouble();
  }

  /* shuts the motor after its no longer needed */
  @Override
  public void close() throws Exception {
    wheelMotor.close();
  }
}
