package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.IDLE_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_VOLTAGE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VELOCITY_TOLERANCE;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.shooter.ShooterConstants.VelocityControl;

public class Shooter extends SubsystemBase implements AutoCloseable {

  private final WheelIO hardware;
  private final SysIdRoutine characterization;
  private double lastVelocityTarget = 0.0;

  private final PIDController controller =
      new PIDController(VelocityControl.P, VelocityControl.I, VelocityControl.D);
  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(VelocityControl.S, VelocityControl.V, VelocityControl.A);

  public Shooter(WheelIO hardware) {
    this.hardware = hardware;
    controller.setTolerance(
        VELOCITY_TOLERANCE.in(RadiansPerSecond)); // how close it needs to be to its target speed

    characterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(10.0), Seconds.of(11)),
            new SysIdRoutine.Mechanism(
                v -> hardware.setVoltage(v.in(Volts)), null, this, "top shooter"));

    SmartDashboard.putData(
        "shooter top quasistatic backward", characterization.quasistatic(Direction.kReverse));
    SmartDashboard.putData(
        "shooter top quasistatic forward", characterization.quasistatic(Direction.kForward));
    SmartDashboard.putData(
        "shooter top dynamic backward", characterization.dynamic(Direction.kReverse));
    SmartDashboard.putData(
        "shooter top dynamic forward", characterization.dynamic(Direction.kForward));

    setDefaultCommand(runShooter(IDLE_VELOCITY.in(RadiansPerSecond)).withName("Idle"));
  }

  public static Shooter create() {
    return Robot.isReal() ? new Shooter(new RealWheel()) : new Shooter(new SimWheel());
  }

  public static Shooter none() {
    return new Shooter(new NoWheel());
  }

  /**
   * @return The value of the velocity (in Radians Per Second)
   */
  @Logged
  public double getVelocity() {
    return hardware.getVelocity();
  }

  /**
   * Updates the velocity setpoint
   *
   * @param velocitySetPoint The velocity setpoint
   */
  public void update(double velocitySetpoint) {
    double velocity =
        MathUtil.clamp(
            velocitySetpoint,
            -MAX_VELOCITY.in(RadiansPerSecond),
            MAX_VELOCITY.in(RadiansPerSecond));
    double ffVolts = ff.calculateWithVelocities(lastVelocityTarget, velocity);
    lastVelocityTarget = velocity;
    double pidVolts = controller.calculate(getVelocity(), velocity);
    hardware.setVoltage(MathUtil.clamp(pidVolts + ffVolts, -MAX_VOLTAGE, MAX_VOLTAGE));
  }

  /**
   * Checks if the PID position is at the velocity setpoint
   *
   * @return PID position is at setpoint
   */
  @Logged
  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  /**
   * Checks if the current velocity and the target velocity is less than the velocity tolerance
   *
   * @param velocity Target velocity
   * @return true or false
   */
  public boolean atVelocity(double velocity) {
    return Math.abs(velocity - getVelocity()) < VELOCITY_TOLERANCE.in(RadiansPerSecond);
  }

  /**
   * @return The setpoint of the Controller
   */
  @Logged
  public double setpoint() {
    return controller.getSetpoint();
  }

  /**
   * Runs shooter at a velcoity
   *
   * @param velocity The velocity as a DoubleSupplier
   * @return the Command to set the shooter's velocity
   */
  public Command runShooter(DoubleSupplier velocity) {
    return run(() -> update(velocity.getAsDouble())).withName("running shooter");
  }

  /**
   * Runs shooter at a specific velocity
   *
   * @param velocity The desired velocity
   * @return Command to set the velocity
   */
  public Command runShooter(double velocity) {
    return runShooter(() -> velocity);
  }

  /** closes motor */
  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
