package org.sciborgs1155.robot.commands;
import org.sciborgs1155.robot.shooter.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleEntry;

import org.sciborgs1155.robot.commands.shooting.FuelVisualizer;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.commands.shooting.MovingShot;
import static org.sciborgs1155.robot.FieldConstants.allianceReflect;
import org.sciborgs1155.robot.drive.Drive;

import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CENTER_TO_SHOOTER;

public class Shooting {

    public static final DoubleEntry LATENCY_TIME = Tuning.entry("/ShootingData/Latency Time", 0.1);
    private final MovingShot movingShot = new MovingShot();

    private final Shooter shooter;
    private final Hood hood;
    private final FuelVisualizer fuelVisualizer;
    private final Drive drive;

    private Translation2d lastTarget = new Translation2d();

    public Shooting(Shooter shooter, Hood hood, FuelVisualizer fuelVisualizer) {
        this.shooter = shooter;
        this.hood = hood;
        this.fuelVisualizer = fuelVisualizer;
        this.drive = drive;
    }
    public record ShooterParams(double RADS, double hoodAngle) {}

    public ShooterParams calculateShot(Translation2d target) {
        //reflects the target 
        Translation2d reflectedTarget = allianceReflect(target);

        //predicted robot pose using latency time
        Pose2d latencyPose =
            drive.pose().exp(drive.robotRelativeChassisSpeeds().toTwist2d(LATENCY_TIME.get()));

        lastTarget = target;

        //shifts the hoodPose using latencyPose
        Pose2d hoodPose = latencyPose.transformBy(
            new Transform2d(
                CENTER_TO_SHOOTER.getTranslation().toTranslation2d(),
                CENTER_TO_SHOOTER.getRotation().toRotation2d()));
        

        ChassisSpeeds speeds = drive.fieldRelativeChassisSpeeds();


    }
    
}
