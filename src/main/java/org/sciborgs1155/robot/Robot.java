package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.lib.LoggingUtils.log;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Arrays;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.Tracer;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Alignment;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class Robot extends CommandRobot {
  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  private final PowerDistribution pdh = new PowerDistribution();

  // SUBSYSTEMS
  Drive drive = new Drive();
  private final Vision vision = Vision.create();

  // COMMANDS
  private final Alignment align = new Alignment(drive);

  @NotLogged private final SendableChooser<Command> autos = Autos.configureAutos(drive);

  @Logged private double speedMultiplier = Constants.FULL_SPEED_MULTIPLIER;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
    configureBindings();

    // Warms up pathfinding commands, as the first run could have significant delays.
    CommandScheduler.getInstance().schedule(align.warmupCommand());
  }

  @Override
  public void robotPeriodic() {
    Tracer.startTrace("commands");
    CommandScheduler.getInstance().run();
    Tracer.endTrace();
  }

  /** Configures basic behavior for different periods during the game. */
  private void configureGameBehavior() {
    // TODO: Add configs for all additional libraries, components, intersubsystem interaction
    // Configure logging with DataLogManager, Epilogue, and FaultLogger
    DataLogManager.start();
    SignalLogger.enableAutoLogging(true);
    addPeriodic(FaultLogger::update, 2);
    Epilogue.bind(this);

    FaultLogger.register(pdh);
    SmartDashboard.putData("Auto Chooser", autos);

    if (TUNING) {
      addPeriodic(
          () ->
              log(
                  "/Robot/camera transforms",
                  Arrays.stream(vision.cameraTransforms())
                      .map(
                          t ->
                              new Pose3d(
                                  drive
                                      .pose3d()
                                      .getTranslation()
                                      .plus(
                                          t.getTranslation()
                                              .rotateBy(drive.pose3d().getRotation())),
                                  t.getRotation().plus(drive.pose3d().getRotation())))
                      .toArray(Pose3d[]::new),
                  Pose3d.struct),
          PERIOD.in(Seconds));
    }

    // Configure pose estimation updates every tick
    addPeriodic(
        () ->
            drive.updateEstimates(
                vision.estimatedGlobalPoses(drive.gyroHeading(), disabled().getAsBoolean())),
        PERIOD);

    RobotController.setBrownoutVoltage(6.0);

    if (isReal()) {
      URCL.start();
      pdh.clearStickyFaults();
      pdh.setSwitchableChannel(true);
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
      addPeriodic(() -> vision.simulationPeriodic(drive.pose()), PERIOD.in(Seconds));
    }
  }

  /** Configures trigger -> command bindings. */
  private void configureBindings() {
    drive.setDefaultCommand(drive.drive(driver::getLeftY, driver::getRightY));
  }

  /**
   * Command factory to make both controllers rumble.
   *
   * @param rumbleType The area of the controller to rumble.
   * @param strength The intensity of the rumble.
   * @return The command to rumble both controllers.
   */
  public Command rumble(RumbleType rumbleType, double strength) {
    return Commands.runOnce(
            () -> {
              driver.getHID().setRumble(rumbleType, strength);
              operator.getHID().setRumble(rumbleType, strength);
            })
        .andThen(Commands.waitSeconds(0.3))
        .finallyDo(
            () -> {
              driver.getHID().setRumble(rumbleType, 0);
              operator.getHID().setRumble(rumbleType, 0);
            });
  }

  public Command systemsCheck() {
    return Commands.sequence(drive.systemsCheck()).withName("Test Mechanisms");
  }

  @Override
  public void close() {
    super.close();
    try {
      drive.close();
    } catch (Exception e) {
    }
  }
}
