package org.sciborgs1155.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;

import java.util.function.DoubleConsumer;

/**
 * Simple Motor that utilizes a {@link DoubleConsumer} for setting power and provides no feedback.
 * Can work with {@link TalonFX}'s and {@link SparkBase}'s. Can be closed with the specified {@link
 * Runnable}.
 */
public class SimpleMotor {
  /** Function for setting motor power. */
  private final DoubleConsumer set;

  /** Function for setting voltage. */
  private final DoubleConsumer setVoltage;

  /** Function for closing the motor. */
  private final Runnable close;

  /**
   * Constructor.
   *
   * @param set : {@link DoubleConsumer} for setting motor power.
   * @param close : {@link Runnable} interface for the motor.
   */
  public SimpleMotor(DoubleConsumer set, DoubleConsumer setVoltage, Runnable close) {
    this.set = set;
    this.close = close;
    this.setVoltage = setVoltage;
  }

  /**
   * @return Returns a new {@link SimpleMotor} that controls a {@link TalonFX} motor. The motor is
   *     registered with {@link TalonUtils} and {@link FaultLogger}.
   * @param motor : {@link TalonFX} controller instance with device ID.
   * @param config : {@link TalonFXConfiguration} to apply to the motor.
   */
  public static SimpleMotor talon(TalonFX motor, TalonFXConfiguration config) {
    FaultLogger.register(motor);
    TalonUtils.addMotor(motor);
    motor.getConfigurator().apply(config);
    return new SimpleMotor(motor::set, motor::setVoltage, motor::close);
  }

  /**
   * 
   * @param motor controller instance with device ID
   * @param config apply config to motor
   * @return a new simplemotor that controls a sparkflex motor registerred with fault logger
   */
  public static SimpleMotor spark(SparkFlex motor, SparkBaseConfig config){
    FaultLogger.register(motor);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return new SimpleMotor(motor::set, motor::setVoltage, motor::close);
  }

  /** Returns a new {@link SimpleMotor} that does absolutely nothing. */
  public static SimpleMotor none() {
    return new SimpleMotor(v -> {}, v -> {}, () -> {});
  }

  /** Passes power into the '{@link #set}' consumer specified in the constructor. */
  public void set(double power) {
    set.accept(power);
  }

  /** Sets voltage. */
  public void setVoltage(double voltage) {
    setVoltage.accept(voltage);
  }

  /** Closes the motor by running the close runnable specified in the constructor. */
  public void close() {
    close.run();
  }
}
