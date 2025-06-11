package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.geometry.Translation2d;

/** Guides the robot towards a point along a given heading */
public class Attractor extends FieldObject
{
  protected double approachHeading;
  /** Point where the approach heading intersects the effect radius */
  protected Translation2d frontCheckpoint;
  /** Point opposite where the approach heading intersects the effect radius */
  protected Translation2d backCheckpoint;

  public Translation2d process(Translation2d controlInput)
  {
    return controlInput;
  }

  public boolean checkPosition()
  {
    return
    (
      centre.getDistance(robotPos) <= radius &&
      frontCheckpoint.getDistance(robotPos) <= backCheckpoint.getDistance(robotPos)
    );
  }
}
