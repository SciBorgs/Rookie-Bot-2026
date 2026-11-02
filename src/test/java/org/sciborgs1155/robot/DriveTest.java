package org.sciborgs1155.robot;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.AfterEach;
import org.sciborgs1155.robot.drive.Drive;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;


public class DriveTest {

    @BeforeEach
    public void setup() {
        setupTests();
        Drive drive = Drive.create();
    }

    @AfterEach
    public void destroy() throws Exception{
        reset(drive);
    }

    @RepeatedTest(5)
    public void testDrive() {
        double val = Math.random();
        double setpoint = val * MAX_ANGLE.in(Radians);
        assertEquals(setpoint, drive.getPosition().in(Radians), DRIVE_TOLERANCE.in(Radians).in(Radians));
    }
}
