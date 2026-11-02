package org.sciborgs1155.robot;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.sciborgs1155.robot.slapdown.Slapdown;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.sciborgs1155.robot.slapdown.SlapdownConstants.*;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;



public class SlapdownTest {
    Slapdown slapdown;


    /**
     * sets up slapdown
     */
    @BeforeEach
    public void setup(){
        setup();
        slapdown = Slapdown.create();
    }

    /** 
     * @throws Exception reset pos
     */

    @AfterEach
    public void destroy() throws Exception{
        reset(slapdown);
    }

    /**
     *  tests if changing orientation works correctly
     */
    @RepeatedTest(5)
    public void orientation(){
        double val = Math.random();
        double setpoint = val * MAX_ANGLE.in(Radians);
        CommandScheduler.getInstance().schedule(slapdown.goTo(setpoint));
        fastForward(Seconds.of(5));
        assertEquals(setpoint, slapdown.position(), POSITION_TOLERANCE.in(Radians));
    }

}
