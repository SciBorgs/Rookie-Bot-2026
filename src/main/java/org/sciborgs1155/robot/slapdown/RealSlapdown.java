package org.sciborgs1155.robot.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.Slapdown.*;
import static org.sciborgs1155.robot.slapdown.SlapdownConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.sciborgs1155.lib.FaultLogger;

public class RealSlapdown implements SlapdownIO {

  SparkFlex motor;

  public RealSlapdown() {
    motor = new SparkFlex(EXTENSION, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();

    config.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    config.idleMode(IdleMode.kBrake);

    config.inverted(false);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(MAX_ANGLE.in(Radians));

    config.encoder.positionConversionFactor(GEARING);

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
    return motor.getEncoder().getPosition() * Math.PI * 2;
  }
}
