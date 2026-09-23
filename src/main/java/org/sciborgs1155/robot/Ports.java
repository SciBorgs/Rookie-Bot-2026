package org.sciborgs1155.robot;

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
    public static final int GYRO_CHANNEL = 1;
  }

  public static final Map<Integer, String> ID_TO_NAME =
      Map.ofEntries(
          Map.entry(Drive.RIGHT_LEADER, "Right Leader"),
          Map.entry(Drive.RIGHT_FOLLOWER, "Right Follower drive"),
          Map.entry(Drive.LEFT_FOLLOWER, "Left_Follower"),
          Map.entry(Drive.LEFT_LEADER, "Left Leader"),
          Map.entry(Drive.GYRO_CHANNEL, "Gyro Channel"));

  public static final class LEDs {
    public static final int LED_PORT = 9;
  }

  /* PLACE HOLDERS FOR NOW */
  public static final class Shooter {
    public static final int WHEEL_MOTOR_LEADER = 2;
    public static final int WHEEL_MOTOR_FOLLOWER = 3;

    public static final int HOOD_MOTOR = 4;
  }
}
