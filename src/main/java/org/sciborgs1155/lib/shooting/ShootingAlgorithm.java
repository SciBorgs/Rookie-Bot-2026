package org.sciborgs1155.lib.shooting;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N2;

public class ShootingAlgorithm {

    /**
     * Make a interpolating tree map (look up table)
     * Key is the distance to target
     * Value is the RPM that is interpolated
     */


    InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

    public ShootingAlgorithm() {

        //PLACE HOLDER VALUES FOR NOW 
        shooterTable.put(1.0, 1500.0);
        shooterTable.put(2.0, 2500.0);
        shooterTable.put(3.0, 3800.0);
    }

    /**
     * Method to return the desired rpm for the hood
     * @param distanceToTarget The distance to target in some unit (Have to figure it out)
     * @return the interpolated RPM using shooterTable, a tree map 
     */
    public double getRPM(double distanceToTarget) {
        return shooterTable.get(distanceToTarget);
    }

    /**
     * Method to find the velocity as a vector (direction and spped) to run the shooter to shoot accurately
     * 
     * @param shooterPosition Positon of shooter as a transaltion3D
     * @param targetPosition Postion of the target as a translation3D
     * @param shooterVelocity current flywheel velocity
     */
    public void autoAim(Translation3d shooterPosition, Translation3d targetPosition, Vector<N2> shooterVelocity) {

        Translation3d displacementVector = targetPosition.minus(shooterPosition);

    }

}
