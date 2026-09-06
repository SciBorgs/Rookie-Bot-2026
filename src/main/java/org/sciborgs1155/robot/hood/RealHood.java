package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Shooter.HOOD_MOTOR;
import static org.sciborgs1155.robot.shooter.ShooterConstants.GEAR_RATIO;
import static org.sciborgs1155.robot.shooter.ShooterConstants.STATOR_CURRENT_LIMIT;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class RealHood implements HoodIO {
  private final SparkFlex hoodMotor;
  private final SparkFlexConfig config;
  private final RelativeEncoder encoder;

  /* Configurations  */
  public RealHood() {

    hoodMotor = new SparkFlex(HOOD_MOTOR, MotorType.kBrushless);

    encoder = hoodMotor.getEncoder();

    config = new SparkFlexConfig();

    config.inverted(true); // will delete this if motor doen't need to be inverted (look at CAD)
    config.smartCurrentLimit((int) STATOR_CURRENT_LIMIT.in(Amps));

    config.idleMode(IdleMode.kBrake);

    hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    double position =
        encoder.getPosition()
            / GEAR_RATIO; // gear ratio to covert motor rotations to physical rotations
    return position * (2 * Math.PI);
  }

  @Override
  public double velocity() {
    return encoder.getVelocity();
  }

  @Override
  public void close() throws Exception {
    hoodMotor.close();
  }
}
