package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.hood.HoodConstants.DEFAULT_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class Hood extends SubsystemBase implements AutoCloseable {

    private final HoodIO hardware;
    private final ProfiledPIDController controller = new ProfiledPIDController(ControlConstants.P, ControlConstants.I, ControlConstants.D, new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCEL.in(RadiansPerSecondPerSecond)));
    private final ArmFeedforward ff = new ArmFeedforward(ControlConstants.S, ControlConstants.G, ControlConstants.V, ControlConstants.A);

    private final SysIdRoutine sysIdRoutine;

    /*Creates either an hardware or sim depending if the robot is real  */
    public static Hood create() {
        return Robot.isReal() ? new Hood(new RealHood()) : new Hood(new SimHood());

    }
    
    public Hood(HoodIO hardware) {
        this.hardware = hardware;

        
    controller.setTolerance(POSITION_TOLERANCE.in(Radians)); //how close it needs to needs to be
    controller.reset(angle());
    setDefaultCommand(goTo(DEFAULT_ANGLE));

    sysIdRoutine =
        new SysIdRoutine(
            new Config(RAMP_RATE, STEP_VOLTAGE, TIME_OUT),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));

    SmartDashboard.putData(
        "Robot/hood/quasistatic forward",
        sysIdRoutine
            .quasistatic(Direction.kForward)
            .until(() -> atPosition(MAX_ANGLE.in(Radians)))
            .withName("hood quasistatic forward"));
    SmartDashboard.putData(
        "Robot/hood/quasistatic backward",
        sysIdRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> atPosition(MIN_ANGLE.in(Radians)))
            .withName("hood quasistatic backward"));
    SmartDashboard.putData(
        "Robot/hood/dynamic forward",
        sysIdRoutine
            .dynamic(Direction.kForward)
            .until(() -> atPosition(MAX_ANGLE.in(Radians)))
            .withName("hood dynamic forward"));
    SmartDashboard.putData(
        "Robot/hood/dynamic backward",
        sysIdRoutine
            .dynamic(Direction.kReverse)
            .until(() -> atPosition(MIN_ANGLE.in(Radians)))
            .withName("hood dynamic backward"));
    }

    /**
     * 
     * @return the angle of the hood
     */
    @Logged
    public double angle() {
        return hardware.getPosition();
    }

    /**
     * 
     * @return the position of setpoint
     */
    @Logged
    public double angleSetpoint() {
        return controller.getSetpoint().position;
    }

    /**
     * 
     * @return the velcoity of the hood 
     */
    @Logged
    public double velocity() {
        return hardware.velocity();
    }

    /**
     * 
     * @return the velocity of the setpoint
     */
    @Logged
    public double getVelocitySetpoint() {
        return controller.getSetpoint().velocity;
    }

    /**
     * Command to set the hood to a angle 
     * 
     * @param goal the desired angle
     * @return command to set hood to goal
     */
    @Logged
    public Command goTo(Angle goal) {
        return goTo(() -> goal.in(Radians));
    }

    /**
     * Command to set hoood to an angle using a double supplier 
     * @param goal The deisred angle as a double supplier
     * @return A command that will set hood to goal
     */
    @Logged
    public Command goTo(DoubleSupplier goal) {
        return run(() -> update(goal.getAsDouble())).withName("Hood GO");
    }

    /**
     * Updadtes hood voltage that will set the hood to the desired position based on ff and pid calculations 
     * @param position The deisred position of the hood
     */
    @Logged
    public void update(double position) {
        double goal = MathUtil.clamp(position, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
        double PIDCalculations = controller.calculate(angle(), goal);
        double ffCalculations = ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity);
        hardware.setVoltage((PIDCalculations + ffCalculations));

    }

    /**
     * 
     * @return Boolean depending if it is at goal 
     */
    @Logged 
    public boolean atGoal() {
        return controller.atGoal();

    }

    /**
     * 
     * @param angle The deisred angle 
     * @return Boollean depending if the abs value of its difference is less than position tolerance
     */
    @Logged
    public boolean atPosition(double angle) {
        return Math.abs(angle - angle()) < POSITION_TOLERANCE.in(Radians);

    }

    @Override
    public void close() throws Exception {

    }
    

}
