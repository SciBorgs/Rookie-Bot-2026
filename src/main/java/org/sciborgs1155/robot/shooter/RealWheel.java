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
import org.sciborgs1155.lib.FaultLogger;

public class RealWheel implements WheelIO {

  private final SparkFlex wheelMotorLeader;
  private final SparkFlex wheelMotorFollower;
  private final SparkFlexConfig config;
  private final RelativeEncoder encoderLeader;

  /* Sets motor configurations */
  public RealWheel() {

    wheelMotorLeader = new SparkFlex(WHEEL_MOTOR_LEADER, MotorType.kBrushless);
    wheelMotorFollower = new SparkFlex(WHEEL_MOTOR_FOLLOWER, MotorType.kBrushless);
    encoderLeader = wheelMotorLeader.getEncoder();

    config = new SparkFlexConfig();
    config.smartCurrentLimit((int) STATOR_CURRENT_LIMIT.in(Amps));
    config.idleMode(IdleMode.kCoast);

    wheelMotorLeader.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(wheelMotorLeader, false); // follower motor automatically follows leader now

    wheelMotorFollower.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // registers the motors to be checked for faults
    FaultLogger.register(wheelMotorLeader);
    FaultLogger.register(wheelMotorFollower);
  }

  @Override
  public void setVoltage(double voltage) {
    wheelMotorLeader.setVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return encoderLeader.getVelocity() * Math.PI * 2;
  }

  /* shuts the motor after its no longer needed */
  @Override
  public void close() throws Exception {
    wheelMotorLeader.close();
    wheelMotorFollower.close();
  }
}
