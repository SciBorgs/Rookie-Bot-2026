package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class HoodVisualizer {
  private final Mechanism2d mech;
  private final MechanismLigament2d hood;
  private final MechanismLigament2d basketBallTrajectory; // visualize basketball
  private final String name;

  /**
   * @param name The name
   * @param hoodColor The color of the hood viusalizer
   * @param fuelTrajColor The color of fuel trajectory
   */
  public HoodVisualizer(String name, Color8Bit hoodColor, Color8Bit basketBallTrajColor) {
    this.name = name;

    mech = new Mechanism2d(100, 100);
    MechanismRoot2d root = mech.getRoot("hood", 5, 0);

    hood =
        root.append(
            new MechanismLigament2d(
                name,
                HoodConstants.HOOD_RADIUS.in(Inches) * 5,
                HoodConstants.STARTING_ANGLE.in(Radians),
                3,
                hoodColor));

    basketBallTrajectory =
        hood.append(new MechanismLigament2d("Fuel Trajectory", 25, 90, 2, basketBallTrajColor));
  }

  /**
   * Sets hood to an angle
   *
   * @param angle The angle to set hood to
   */
  public void setAngle(double angle) {
    hood.setAngle(angle);
    SmartDashboard.putData("/Robot/hood/" + name, mech);
  }
}
