package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Shooter.HOOD_MOTOR;
import static org.sciborgs1155.robot.shooter.ShooterConstants.GEAR_RATIO;
import static org.sciborgs1155.robot.shooter.ShooterConstants.STATOR_CURRENT_LIMIT;
import static org.sciborgs1155.robot.shooter.ShooterConstants.SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealHood implements HoodIO {
  private final TalonFX hoodMotor;

  /* Configurations  */
  public RealHood() {

    hoodMotor = new TalonFX(HOOD_MOTOR);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // counterclockwise is postive
    configs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT.in(Amps);
    configs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);

    hoodMotor.getConfigurator().apply(configs);

    /* Checks the motors */
    FaultLogger.register(hoodMotor);

    /* adds motors to a list of all global motors */
    TalonUtils.addMotor(hoodMotor);
  }

  @Override
  public void setHoodVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }

  @Override
  public double getHoodPosition() {
    var currentSig = hoodMotor.getPosition();
    currentSig.refresh();
    var convertedSig =
        currentSig.getValueAsDouble()
            / GEAR_RATIO; // gear ratio to covert motor rotations to physical rotations
    return convertedSig * (2 * Math.PI);
  }

  @Override
  public double velocity() {
    return hoodMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    hoodMotor.close();
  }
}
