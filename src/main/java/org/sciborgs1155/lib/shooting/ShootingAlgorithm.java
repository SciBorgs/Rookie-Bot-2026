package org.sciborgs1155.lib.shooting;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N2;

public class ShootingAlgorithm {

    /* Interpolating tree map, where key is the distance to target and value is the RPM*/
    InterpolatingDoubleTreeMap RPMTable = new InterpolatingDoubleTreeMap();

    /*Interpolating tree map, where key is the distance to target, and value is the time of fight in seconds */
    InterpolatingDoubleTreeMap ToF = new InterpolatingDoubleTreeMap();

    public ShootingAlgorithm() {

        /* Both tables have made up values for now */

        RPMTable.put(1.0, 1500.0);
        RPMTable.put(2.0, 2500.0);
        RPMTable.put(3.0, 3800.0);

        ToF.put(3.0, 0.5);
        ToF.put(9.0, 0.7);
        ToF.put(18.0, 1.0);

    }

    /**
     * Method to return the desired rpm for the hood
     * @param distanceToTarget The distance to target in some unit (Have to figure it out)
     * @return the interpolated RPM using shooterTable, a tree map 
     */
    public double getRPM(double distanceToTarget) {
        return RPMTable.get(distanceToTarget);
    }

    public record ShootingShot(double virtualDistance, double drivetrainAngle) {}

    /**
     * Method to find the distance to target, and drive train angle
     * 
     * @param shooterPosition Positon of shooter as a transaltion3D
     * @param targetPosition Postion of the target as a translation3D
     * @param shooterVelocity current velocity
     */
    public ShootingShot shootWhileMoving(Translation3d shooterPosition, Translation3d targetPosition, Vector<N2> velocity) {

        double targetX = targetPosition.getX();
        double targetY = targetPosition.getY();

        double robotX = shooterPosition.getX();
        double robotY = shooterPosition.getY();

        
        double distanceToTarget = targetPosition.getDistance(shooterPosition);

        double velocityX = velocity.get(0,0); 
        double velocityY = velocity.get(1, 0);


        double VirtualTargetX = 0;
        double VirtualTargetY = 0;

        for (int i=0; i < 3; i++) {
            double T = ToF.get(distanceToTarget);

            double DriftX = velocityX * T;
            double DriftY = velocityY * T;

            VirtualTargetX = targetX - DriftX;
            VirtualTargetY =  targetY - DriftY;

            Translation3d VirtualTarget = new Translation3d(VirtualTargetX, VirtualTargetY, targetPosition.getZ());

            distanceToTarget = VirtualTarget.getDistance(shooterPosition);
        }


        double finalAngle = Math.atan2(VirtualTargetY- robotY, VirtualTargetX - robotX);

        return new ShootingShot(distanceToTarget, finalAngle);

    }

}
