package frc.robot.util.controlTransmutation;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public abstract class GeoFence extends FieldObject
{
  // Inherits from FieldObject: T2D centre, double radius, double buffer
  protected ArrayList<Attractor> attractors;

  public Translation2d process(Translation2d controlInput)
  {
    if (!(checkPosition() || checkAttractors()))
      {return controlInput;}
    
    return dampMotion(controlInput, robotPos, robotRadius);
  }

  protected boolean checkPosition()
    {return true;}

  protected boolean checkAttractors()
  {
    if (attractors.size() == 0)
      {return false;}

    for (int i = 0; i < attractors.size(); i++)
    {
      if (attractors.get(i).checkPosition())
        {return true;}
    }

    return false;
  }
  
  protected Translation2d dampMotion(Translation2d motionXY, Translation2d robotXY, double robotR)
  {
    return motionXY;
  }

  /**
   * Point type GeoFence object </p>
   * Defined as a single point with a radius
   */
  public class Point extends GeoFence
  {
    
  }

  /**
   * Fence type GeoFence object </p>
   * The outer wall that the robot must stay within </p>
   * A cardinal rectangular region defined by two corners
   */
  public class Fence extends GeoFence
  {
    
  }

  /**
   * Box type GeoFence object </p>
   * A cardinal rectangular region defined by two corners
   */
  public class Box extends GeoFence
  {
    
  }

  /**
   * Line type GeoFence object </p>
   * Defined between two points </p>
   * Note: causes edge-case behaviours when meeting other objects at acute angles
   */
  public class Line extends GeoFence
  {
    
  }

  /**
   * Polygon type GeoFence object </p>
   * A rotated regular polygon built from a series of Line objects </p>
   * with handling to only process the nearest line
   */
  public class Polygon extends GeoFence
  {
    
  }
}
