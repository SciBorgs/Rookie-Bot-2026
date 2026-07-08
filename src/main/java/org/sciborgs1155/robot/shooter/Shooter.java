package org.sciborgs1155.robot.shooter;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.shooter.ShooterConstants.VelocityControl;

public class Shooter extends SubsystemBase {

    private final WheelIO hardware;

    private final PIDController controller = new PIDController(VelocityControl.P, VelocityControl.I, VelocityControl.D);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(VelocityControl.S, VelocityControl.V, VelocityControl.A);

    public Shooter(WheelIO hardware) {
        this.hardware = hardware;
    }

    public static Shooter create() {
        return Robot.isReal() ? new Shooter(new RealWheel()) : new Shooter(new SimWheel());
    }

}
