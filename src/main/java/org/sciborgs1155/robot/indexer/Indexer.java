package org.sciborgs1155.robot.indexer;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.sciborgs1155.lib.Beambreak;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.Ports.BeambreakPorts;
import org.sciborgs1155.robot.Ports.IndexerPorts; 


public class Indexer extends SubsystemBase{
    private final SimpleMotor indexerMotor;



    private final Beambreak beambreak;
    public final Trigger blocked;

    public static Indexer create() {
        return Robot.isReal() ? new Indexer(realIndexerMotor(), Beambreak.real(BeambreakPorts.BEAMBREAK)) : noIndexer();
    }

    public static Indexer noIndexer(){
        return new Indexer(SimpleMotor.none(), Beambreak.none());
    }


    private Indexer(SimpleMotor indexerMotor, Beambreak beambreak){
        this.indexerMotor = indexerMotor;
        this.beambreak = beambreak;
        this.blocked = new Trigger(() -> !beambreak.getState());

    }

    public static SimpleMotor realIndexerMotor() {
        TalonFX talonIndexer = new TalonFX(IndexerPorts.INDEXER_MOTOR);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = IndexerConstants.CURRENT_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return SimpleMotor.talon(talonIndexer, config);
    }

    
    public Command runIndexer(double power) {
        return run(() -> indexerMotor.set(power));
    }


    //power commands using runIndexer


    public Command stop(){
        return runIndexer(0);
    }
    public Command forward(){
        return runIndexer(1);
    }
    public Command reverse(){
        return runIndexer(-1);
    }
    public Command set(double power){
        double clampedPower = MathUtil.clamp(power, -1.0, 1.0);
        return runIndexer(clampedPower * IndexerConstants.INDEXER_MAXPOWER); //scale by maxpower so it stays linear but also clamped
    }


    //new

    public Command forwardUntilBlocked(){
        return forward().until(blocked);
    }

    public Command unJamDefault() //stop reversing after a period of time
    {
        return reverse()
            .withTimeout(0.5);
    }

    public Command unJamSensor(){ //stop reversing once beambreak isn't blocked
        return reverse()
            .onlyWhile(blocked);
    }

    public Command smartUnJam(){ //if sensor is unblocked OR time has passed stop reversing
        Command raceCommand = 
            unJamSensor()
            .raceWith(unJamDefault())
            .andThen(stop());
            
        return raceCommand;
    }

}
