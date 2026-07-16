package org.sciborgs1155.robot.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Robot;

public class Shooter extends SubsystemBase {

  WheelIO hardware;

  public Shooter(WheelIO hardware) {
    this.hardware = hardware;
  }

  public static Shooter create() {
    return Robot.isReal() ? new Shooter(new RealWheel()) : new Shooter(new SimWheel());
  }
}
