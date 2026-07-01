package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.lib.LoggingUtils.log;
import static org.sciborgs1155.robot.FieldConstants.allianceFromPose;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.Supplier;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.FaultLogger.Fault;
import org.sciborgs1155.lib.FaultLogger.FaultType;
import org.sciborgs1155.lib.RepulsorFieldPlanner;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;

public final class help
{
  public Command practice_command()
  {
    return run(coroutine -> {
      
      SmartDashboard.putNumber("PracticeCmd/Cycles:", 0); //not a gd reference
      SmartDashboard.putBoolean("PracticeCmd/Running:", true);

      for(int x = 0; x < 45; x++){
        double read = SmartDashboard.getNumber("PracticeCmd/Cycles", -1); //does nothing for practice
        SmartDashboard.putNumber("PracticeCmd/Cycles:", x);
        coroutine.waitFor(200);
        coroutine.yield();
      }
    
      SmartDashboard.putBoolean("PracticeCmd/Running:", false);

   
  }).named("practice");
}
