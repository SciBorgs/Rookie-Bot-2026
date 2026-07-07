package org.sciborgs1155.robot.shooter;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    RealShooter RealShooter;
    SimShooter SimShooter;

    public static Shooter create() {
        return Robot.isReal() ? new Shooter(new RealShooter()) : new Shooter(new SimShooter());

    }

    /* Constructors */
    public Shooter(RealShooter realShooter) {
        this.RealShooter = realShooter;
    }

    public Shooter(SimShooter SimShooter) {
        this.SimShooter = SimShooter;
    }

    

}
