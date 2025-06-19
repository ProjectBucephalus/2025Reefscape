package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Guides the robot towards a point along a given heading */
public class Attractor extends FieldObject
{
  protected double approachHeading;
  protected Rotation2d approachHeadingRotation;
  /** Point where the approach heading intersects the effect radius */
  protected Translation2d frontCheckpoint;
  /** Point opposite where the approach heading intersects the effect radius */
  protected Translation2d backCheckpoint;

  /**
   * Pulls the robot into the target point along the given heading if the control input is within a certain tollerance
   * @param X x-coordinate of the target
   * @param Y y-coordinate of the target
   * @param approachHeading direction the robot should move to approach the target, degrees anticlockwise
   * @param effectRadius distance from target where the robot will start being attracted
   * @param targetBuffer distance from the target where the robot must be traveling along the approach heading
   */
  public Attractor(double X, double Y, double approachHeading, double effectRadius, double targetBuffer)
  {
    centre = new Translation2d(X, Y);
    this.approachHeading = approachHeading;
    radius = effectRadius;
    buffer = targetBuffer;

    approachHeadingRotation = Rotation2d.fromDegrees(approachHeading);

    frontCheckpoint = centre.minus(new Translation2d(radius, approachHeadingRotation));
    backCheckpoint  = centre.plus(new Translation2d(radius, approachHeadingRotation));


  }

  public Translation2d process(Translation2d controlInput)
  {
    if (controlInput.equals(Translation2d.kZero) || !checkPosition())
      {return controlInput;}

    if (centre.getDistance(robotPos) <= buffer)
    {
      
    }
    

    return controlInput;
  }

  public boolean checkPosition()
  {
    return
    (
      centre.getDistance(robotPos) <= buffer ||
      (
        centre.getDistance(robotPos) <= radius &&
        frontCheckpoint.getDistance(robotPos) <= backCheckpoint.getDistance(robotPos)
      )
    );
  }

  @Override
  public double getDistance()
  {
    return centre.getDistance(robotPos) - (robotRadius);
  }

}
