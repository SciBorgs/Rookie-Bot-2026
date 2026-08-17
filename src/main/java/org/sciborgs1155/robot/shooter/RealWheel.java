package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Shooter.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class RealWheel implements WheelIO {

  private final SparkFlex wheelMotor;
  private final SparkFlexConfig config;
  private final RelativeEncoder encoder;

  /* Sets motor configurations */
  public RealWheel() {

    wheelMotor = new SparkFlex(WHEEL_MOTOR, MotorType.kBrushless);
    encoder = wheelMotor.getEncoder();

    config = new SparkFlexConfig();

    config.inverted(true);
    config.smartCurrentLimit(
        (int) STATOR_CURRENT_LIMIT.in(Amps)); // Limits motor output/stator current

    config.idleMode(IdleMode.kCoast);

    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double voltage) {
    wheelMotor.setVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity() * Math.PI * 2;
  }

  /* shuts the motor after its no longer needed */
  @Override
  public void close() throws Exception {
    wheelMotor.close();
  }
}
