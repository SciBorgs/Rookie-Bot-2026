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
     * Method to find the velocity as a vector (direction and spped) to run the shooter to shoot accurately
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

        double tempTimeOfFlight = ToF.get(distanceToTarget);

        double velocityX = velocity.get(0,0); 
        double velocityY = velocity.get(1, 0);

        double tempDriftX = velocityX * tempTimeOfFlight;
        double tempDriftY = velocityY * tempTimeOfFlight;

        double tempVirtualTargetX = targetX - tempDriftX;
        double tempVirtualTargetY =  targetY - tempDriftY;

        Translation3d tempVirtualTarget = new Translation3d(tempVirtualTargetX, tempVirtualTargetY, targetPosition.getZ());

        double distanceToVirtualTarget = tempVirtualTarget.getDistance(shooterPosition);

        double timeOfFlight = ToF.get(distanceToVirtualTarget);

        double driftX = velocityX * timeOfFlight;
        double driftY = velocityY * timeOfFlight;

        double finalVirtualTargetX = targetX - driftX;
        double finalVirtualTargetY = targetY - driftY;


        double finalAngle = Math.atan2(finalVirtualTargetY- robotY, finalVirtualTargetX - robotX);
        double finalVirtualDistance = new Translation3d(finalVirtualTargetX, finalVirtualTargetY, targetPosition.getZ()).getDistance(shooterPosition);

        return new ShootingShot(finalVirtualDistance, finalAngle);

    }

}
