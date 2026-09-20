package org.sciborgs1155.robot.indexer;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public class IndexerConstants {
    public static final Current CURRENT_LIMIT = Amps.of(40);
    public static final double INDEXER_MAXPOWER = 0.85;
    public static final double GEAR_RATIO = 1; //Placeholder, gear ratio is likely not 1:1

    //public static final int hoursWasted = 3;

}
