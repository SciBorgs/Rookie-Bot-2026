package org.sciborgs1155.robot;

import static java.util.Map.entry;

import java.util.Map;

public final class Ports {
  // TODO: Add and change all ports as needed.


  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

 public static final class Drive {
    public static final int RIGHT_LEADER = 2;
    public static final int RIGHT_FOLLOWER = 3;
    public static final int LEFT_LEADER = 4;
    public static final int LEFT_FOLLOWER = 5;
  
  }


  public static final class LEDs {
    public static final int LED_PORT = 9;
  }

  /* PLACE HOLDERS FOR NOW */
  public static final class Shooter {
    public static final int WHEEL_MOTOR = 2;
    public static final int HOOD_MOTOR = 3;
  }
}
