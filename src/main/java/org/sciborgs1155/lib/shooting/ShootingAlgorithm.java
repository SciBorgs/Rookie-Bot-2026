package org.sciborgs1155.lib.shooting;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N2;

public class ShootingAlgorithm {

  /** Interpolating tree map key is the distance to target; value is the RPM */
  InterpolatingDoubleTreeMap RPMTable = new InterpolatingDoubleTreeMap();

  /** Interpolating tree map key is the distance to target; value is the Time of Flight */
  InterpolatingDoubleTreeMap ToF = new InterpolatingDoubleTreeMap();

  /** Interpolating tree map key is the distance to target; Value is Hood Angle */
  InterpolatingDoubleTreeMap HoodAngleTable = new InterpolatingDoubleTreeMap();

  public ShootingAlgorithm() {

    // Tables have made up values for now

    RPMTable.put(1.0, 1500.0);
    RPMTable.put(2.0, 2500.0);
    RPMTable.put(3.0, 3800.0);

    ToF.put(3.0, 0.5);
    ToF.put(9.0, 0.7);
    ToF.put(18.0, 1.0);

    HoodAngleTable.put(1.0, 0.3);
    HoodAngleTable.put(3.0, 0.5);
    HoodAngleTable.put(9.0, 0.7);
  }

  /**
   * Method to return the desired rpm for the hood
   *
   * @param distanceToTarget The distance to target in some unit (Have to figure it out)
   * @return the interpolated RPM using shooterTable, a tree map
   */
  public double getRPM(double distanceToTarget) {
    return RPMTable.get(distanceToTarget);
  }

  /**
   * Method to return the desired hood angle for the hood
   *
   * @param distanceToTarget The distance to target in some unit (Have to figure it out)
   * @return the interpolated hood angle using HoodTable, a tree m
   */
  public double getHoodAngle(double distanceToTarget) {
    return HoodAngleTable.get(distanceToTarget);
  }

  // To hold the values
  public record ShootingShot(double RPM, double drivetrainAngle, double HaveoodAngle) {}

  /**
   * Method to find the rpm, drivetrain angle, hood angle
   *
   * @param shooterPosition Positon of shooter as a Transaltion3D
   * @param targetPosition Postion of the target as a Translation3D
   * @param shooterVelocity current velocity
   */
  public ShootingShot shootWhileMoving(
      Translation3d shooterPosition, Translation3d targetPosition, Vector<N2> velocity) {

    double targetX = targetPosition.getX();
    double targetY = targetPosition.getY();

    double robotX = shooterPosition.getX();
    double robotY = shooterPosition.getY();

    double distanceToTarget = targetPosition.getDistance(shooterPosition);

    double velocityX = velocity.get(0, 0);
    double velocityY = velocity.get(1, 0);

    double VirtualTargetX = 0;
    double VirtualTargetY = 0;

    for (int i = 0; i < 3; i++) {
      double T = ToF.get(distanceToTarget);

      double DriftX = velocityX * T;
      double DriftY = velocityY * T;

      VirtualTargetX = targetX - DriftX;
      VirtualTargetY = targetY - DriftY;

      Translation3d VirtualTarget =
          new Translation3d(VirtualTargetX, VirtualTargetY, targetPosition.getZ());

      distanceToTarget = VirtualTarget.getDistance(shooterPosition);
    }

    double RPM = getRPM(distanceToTarget);
    double hoodAngle = getHoodAngle(distanceToTarget);

    double finalAngle = Math.atan2(VirtualTargetY - robotY, VirtualTargetX - robotX);

    return new ShootingShot(RPM, finalAngle, hoodAngle);
  }
}
