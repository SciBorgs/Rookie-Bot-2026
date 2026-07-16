package org.sciborgs1155.lib.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootingAlgorithm {

    /**
     * THis is a lookup table
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

    public double getRPM(double distanceToTarget) {
        return shooterTable.get(distanceToTarget);

    }

}
