package org.sciborgs1155.robot.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import org.sciborgs1155.robot.Ports.Drive.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.sciborgs1155.robot.Ports.Drive.LEFT_FOLLOWER;
import static org.sciborgs1155.robot.Ports.Drive.LEFT_LEADER;
import static org.sciborgs1155.robot.Ports.Drive.RIGHT_FOLLOWER;
import static org.sciborgs1155.robot.Ports.Drive.RIGHT_LEADER;

import org.sciborgs1155.robot.Ports;

public class Drive extends SubsystemBase {

  private final SparkFlex rightLeader = new SparkFlex(RIGHT_LEADER, MotorType.kBrushless);
  private final SparkFlex rightFollower = new SparkFlex(RIGHT_FOLLOWER, MotorType.kBrushless);
  private final SparkFlex leftLeader = new SparkFlex(LEFT_LEADER, MotorType.kBrushless);
  private final SparkFlex leftFollower = new SparkFlex(LEFT_FOLLOWER, MotorType.kBrushless);

   public Drive() {
    SparkFlexConfig globalConfig = new SparkFlexConfig(); //for every motor (universal)
    SparkFlexConfig rightLeaderConfig = new SparkFlexConfig();
    SparkFlexConfig leftLeaderConfig = new SparkFlexConfig();
    SparkFlexConfig leftFollowerConfig = new SparkFlexConfig();
    SparkFlexConfig rightFollowerConfig = new SparkFlexConfig();

    globalConfig.idleMode(IdleMode.kBrake);

    leftFollowerConfig.apply(globalConfig).follow(Ports.Drive.LEFT_LEADER);
    rightFollowerConfig.apply(globalConfig).follow(Ports.Drive.RIGHT_LEADER);
    leftLeaderConfig.apply(globalConfig).inverted(true);
    rightLeaderConfig.apply(globalConfig).inverted(false);

    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

}