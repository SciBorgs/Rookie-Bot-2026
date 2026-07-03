package org.sciborgs1155.robot.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/* TO DO:
 * Fix motor configurations
 * work on getHoodPosition
 */
public class RealShooter implements ShooterIO {

  private final TalonFX wheelMotor;
  private final TalonFX hoodMotor;

  /* Sets motor configurations */
  public RealShooter() {

    wheelMotor = new TalonFX(1);
    hoodMotor = new TalonFX(0);

    MotorOutputConfigs configs = new MotorOutputConfigs();

    configs.NeutralMode = NeutralModeValue.Brake;
    configs.Inverted = InvertedValue.CounterClockWise_Postive;

    wheelMotor.getConfigurator().apply(configs);
    hoodMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setFlyWheelVoltage(double voltage) {
    wheelMotor.setVoltage(voltage);
  }

  @Override
  public void setHoodVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }

  @Override
  public double getFlyWheelVelocity() {
    return wheelMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public double getHoodPosition() {
    return null;
  }
}
