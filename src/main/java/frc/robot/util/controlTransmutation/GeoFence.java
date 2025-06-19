package frc.robot.util.controlTransmutation;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;

/** Add your docs here. */
public abstract class GeoFence extends FieldObject
{
  // Inherits from FieldObject: T2D centre, double radius, double buffer, double checkRadius
  protected ArrayList<Attractor> attractors;

  public Translation2d process(Translation2d controlInput)
  {
    if (!(checkPosition() || checkAttractors()))
      {return controlInput;}
    
    return dampMotion(controlInput);
  }

  protected boolean checkAttractors()
  {
    if (attractors.size() > 0)
    {
      for (int i = 0; i < attractors.size(); i++)
      {
        if (attractors.get(i).checkPosition())
          {return true;}
      }
    }

    return false;
  }
  
  protected Translation2d dampMotion(Translation2d motionXY)
  {
    return motionXY;
  }

  protected Translation2d pointDamping(double pointX, double pointY, Translation2d motionXY)
  {
    // Calculates X and Y distances to the point
    double distanceX = pointX - robotPos.getX();
    double distanceY = pointY - robotPos.getY();
    // Calculates the normal distance to the corner through pythagoras; this is the actual distance between the robot and point
    double distanceN = Math.hypot(distanceX, distanceY);
    // Calculates the robot's motion normal and tangent to the point; i.e., towards and away from the point, and from side to side relative to the point
    double motionN   = ((distanceX * motionXY.getX()) + (distanceY * motionXY.getY())) / distanceN;
    double motionT   = ((distanceX * motionXY.getY()) - (distanceY * motionXY.getX())) / distanceN;
    
    // Clamps the normal motion, i.e. motion towards the point, in order to clamp robot speed
    // Sets maximum input towards the object as:
    //      (position within the buffer normalised to [0..1])   *   (angle normalisation factor [1..sqrt(2)])
    //         (dNormal - object radii)[0..buffer] / buffer     *      (mNormal / max(|X|,|Y|))
    motionN = Math.min(motionN, motionN * Conversions.clamp(distanceN-(robotRadius + radius), 0, buffer)
                                    / (Math.max(Math.abs(distanceX),Math.abs(distanceY)) * buffer));
    
    // Converts clamped motion from normal back to X and Y
    double motionX   = ((motionN * distanceX) - (motionT * distanceY)) / distanceN;
    double motionY   = ((motionN * distanceY) + (motionT * distanceX)) / distanceN;
    return new Translation2d(motionX, motionY);
  }
}
