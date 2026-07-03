package org.sciborgs1155.robot.shooter;


import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;


public class RealShooter implements ShooterIO {


  private final TalonFX wheelMotor;
  private final TalonFX hoodMotor;


  /* Sets motor configurations */
  public RealShooter() {


    wheelMotor = new TalonFX(1);
    hoodMotor = new TalonFX(0);


    TalonFXConfiguration configs = new TalonFXConfiguration();


    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // stops the motor
    configs.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // counterclockwise is postive
    configs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT.in(Amps);
    configs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);


    wheelMotor.getConfigurator().apply(configs);
    hoodMotor.getConfigurator().apply(configs);


    /* Checks the motors */
    FaultLogger.register(wheelMotor);
    FaultLogger.register(hoodMotor);


    /* adds motors to a list of all global motors */
    TalonUtils.addMotor(wheelMotor);
    TalonUtils.addMotor(hoodMotor);
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
    var currentSig = hoodMotor.getPosition();
    currentSig.refresh();
    var convertedSig =
        currentSig.getValueAsDouble()
            / GEAR_RATIO; // gear ratio to covert motor rotations to physical rotations
    return convertedSig * (2 * Math.PI);
  }
}
