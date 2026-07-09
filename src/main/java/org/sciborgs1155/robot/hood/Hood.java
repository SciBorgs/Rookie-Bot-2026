package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.hood.HoodConstants.DEFAULT_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.hood.HoodConstants.POSITION_TOLERANCE;
import static org.sciborgs1155.robot.hood.HoodConstants.RAMP_RATE;
import static org.sciborgs1155.robot.hood.HoodConstants.STEP_VOLTAGE;
import static org.sciborgs1155.robot.hood.HoodConstants.TIME_OUT;

import java.util.function.DoubleSupplier;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import org.sciborgs1155.robot.hood.HoodConstants.*;
import org.sciborgs1155.robot.shooter.ShooterConstants.VelocityControl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Angle;



public class Hood extends SubsystemBase implements AutoCloseable {

    private final HoodIO hardware;
    private final ProfiledPIDController controller = new ProfiledPIDController(VelocityControl.P, VelocityControl.I, VelocityControl.D, new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCEL.in(RadiansPerSecondPerSecond)));
    private final ArmFeedforward ff = new ArmFeedforward(VelocityControl.S, VelocityControl.G, VelocityControl.V, VelocityControl.A);

    private final SysIdRoutine sysIdRoutine;
    public static Hood create() {
        return Robot.isReal() ? new Hood(new RealHood()) : new Hood(new SimHood());

    }
    
    public Hood(HoodIO hardware) {
        this.hardware = hardware;

        
    controller.setTolerance(POSITION_TOLERANCE.in(Radians));
    controller.reseta(angle()); //fix
    setDefaultCommand(goTo(DEFAULT_ANGLE));

        sysIdRoutine =
        new SysIdRoutine(
            new Config(RAMP_RATE, STEP_VOLTAGE, TIME_OUT),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));
    }

    @Logged
    public double angle() {
        return hardware.getPosition()
    }

    @Logged
    public Command goTo(Angle goal) {
        return goTo(() -> goal.in(Radians));
    }

    @Logged
    public Command goTo(DoubleSupplier goal) {
        return run(() -> )

    }
    @Override
    public void close() throws Exception {

    }
    


}
