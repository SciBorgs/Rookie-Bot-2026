package org.sciborgs1155.robot.commands;
import org.sciborgs1155.robot.shooter.Shooter;

import edu.wpi.first.math.geometry.Translation2d;

import org.sciborgs1155.robot.commands.shooting.FuelVisualizer;
import org.sciborgs1155.robot.hood.Hood;
import org.sciborgs1155.robot.commands.shooting.MovingShot;
public class Shooting {

    private final MovingShot movingShot = new MovingShot();

    private final Shooter shooter;
    private final Hood hood;
    private final FuelVisualizer fuelVisualizer;
    //TODO: ADD INTAKE LATER

    public Shooting(Shooter shooter, Hood hood, FuelVisualizer fuelVisualizer) {
        this.shooter = shooter;
        this.hood = hood;
        this.fuelVisualizer = fuelVisualizer;
    }

    //void for now 
    public void calculateShot(Translation2d target) {

    }
    
}
