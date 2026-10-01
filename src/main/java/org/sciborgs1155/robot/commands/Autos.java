package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.sciborgs1155.robot.Constants.Robot.MASS;
import static org.sciborgs1155.robot.Constants.Robot.MOI;
import static org.sciborgs1155.robot.Constants.alliance;
import static org.sciborgs1155.robot.drive.DriveConstants.GEARING;
import static org.sciborgs1155.robot.drive.DriveConstants.MAX_SPEED;
import static org.sciborgs1155.robot.drive.DriveConstants.STATOR_LIMIT;
import static org.sciborgs1155.robot.drive.DriveConstants.TRACK_WIDTH;
import static org.sciborgs1155.robot.drive.DriveConstants.WHEEL_COF;
import static org.sciborgs1155.robot.drive.DriveConstants.WHEEL_RADIUS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.drive.Drive;

public class Autos {
  @NotLogged
  public static SendableChooser<Command> configureAutos(Drive drive) {
    AutoBuilder.configure(
        drive::pose,
        drive::resetOdometry,
        drive::robotRelativeChassisSpeeds,
        (s, feedforwards) -> drive.setChassisSpeeds(s),
        new PPLTVController(0.2, MAX_SPEED.in(MetersPerSecond)), // dt is arbitary
        new RobotConfig(
            MASS,
            MOI,
            new ModuleConfig(
                WHEEL_RADIUS,
                MAX_SPEED,
                WHEEL_COF,
                DCMotor.getNeoVortex(1).withReduction(GEARING),
                STATOR_LIMIT,
                1),
            TRACK_WIDTH),
        () -> alliance() == Alliance.Red,
        drive);

    NamedCommands.registerCommand("example", Commands.run(() -> {}));

    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    chooser.addOption("no auto", Commands.none());
    return chooser;
  }
}
