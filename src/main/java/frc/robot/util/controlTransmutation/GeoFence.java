package frc.robot.util.controlTransmutation;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Conversions;
import frc.robot.util.GeoFenceObject;
import frc.robot.util.GeoFenceObject.ObjectTypes;

/** Add your docs here. */
public abstract class GeoFence extends FieldObject
{
  // Inherits from FieldObject: T2D centre, double radius, double buffer
  protected ArrayList<Attractor> attractors;

  public Translation2d process(Translation2d controlInput)
  {
    if (!(checkPosition() || checkAttractors()))
      {return controlInput;}
    
    return dampMotion(controlInput);
  }

  protected boolean checkPosition()
  {
    return centre.getDistance(robotPos) <= radius + buffer + robotRadius;
  }

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

  public double getDistance()
  {
    return centre.getDistance(robotPos) - (radius + robotRadius);
  }

  public Translation2d getCentre()
  {
    return centre;
  }

  /**
   * Point type GeoFence object </p>
   * Defined as a single point with a radius
   */
  public class Point extends GeoFence
  {
    public Point(double x, double y, double radius, double buffer)
    {
      centre = new Translation2d(x, y);
      this.radius = Math.max(radius, minRadius);
      this.buffer = Math.max(buffer, minBuffer);
    }

    public Point(double x, double y)
    {
      this(x, y, minRadius, minBuffer);
    }

    @Override
    protected Translation2d dampMotion(Translation2d motionXY)
    {
      return pointDamping(centre.getX(), centre.getY(), motionXY);
    }
  }

  /**
   * Fence type GeoFence object </p>
   * The outer wall that the robot must stay within </p>
   * A cardinal rectangular region defined by two corners
   */
  public class Fence extends GeoFence
  {
    private double Xa;
    private double Ya;
    private double Xb;
    private double Yb;

    public Fence(double Xa, double Ya, double Xb, double Yb, double radius, double buffer)
    {
      this.Xa = Xa;
      this.Ya = Ya;
      this.Xb = Xb;
      this.Yb = Yb;

      this.radius = radius;
      this.buffer = buffer;

      centre = new Translation2d((Xa + Xb)/2, (Ya + Yb)/2);
    }

    public Fence(double Xa, double Ya, double Xb, double Yb)
    {
      this(Xa, Ya, Xb, Yb, minRadius, minBuffer);
    }

    @Override
    public double getDistance()
    {
      return Math.abs
      (
        Math.min
        (
          Math.min(robotPos.getX() - (Xa + radius), (Xb - radius) - robotPos.getX()),
          Math.min((Yb - radius) - robotPos.getY(), robotPos.getY() - (Ya + radius))
        )
      );
    }

    @Override
    protected boolean checkPosition()
    {
      return 
      (
        (robotPos.getX() >= Xb - (radius + buffer + robotRadius)) ||  // Close to inside of +X barrier
        (robotPos.getX() <= Xa + (radius + buffer + robotRadius)) ||  // Close to inside of -X barrier
        (robotPos.getY() >= Yb - (radius + buffer + robotRadius)) ||  // Close to inside of +Y barrier
        (robotPos.getY() <= Ya + (radius + buffer + robotRadius))     // Close to inside of -Y barrier
      );
    }

    @Override
    protected Translation2d dampMotion(Translation2d motionXY)
    {
      // Calculates distance to the relevant edge of the field
      // Calculates edge position, and subtracts robot position + radius from edge position.

      // Sets the motion in the relevant direction to the minimum of the current motion
      // And the distance from the edge clamped between 0 and the edge buffer, and normalised to a maximum of 1.
      // This ensures the motion in that direction does not go above the clamped + normalised distance from the edge, to cap speed.
      
      double motionX = motionXY.getX();
      double motionY = motionXY.getY();
      double distanceToEdgeX;
      double distanceToEdgeY;
      
      if (motionX > 0)
      {   
        distanceToEdgeX = (Xb - radius) - (robotPos.getX() + robotRadius); 
        motionX = Math.min(motionX, (Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
      }
      else if (motionX < 0)
      {   
        distanceToEdgeX = (robotPos.getX() - robotRadius) - (Xa + radius);
        motionX = Math.max(motionX, (-Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
      }

      if (motionY > 0)
      {   
        distanceToEdgeY = (Yb - radius) - (robotPos.getY() + robotRadius);
        motionY = Math.min(motionY, (Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
      }
      else if (motionY < 0)
      {   
        distanceToEdgeY = (robotPos.getY() - robotRadius) - (Ya + radius);
        motionY = Math.max(motionY, (-Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
      }
      return new Translation2d(motionX, motionY);
    }
  }

  /**
   * Box type GeoFence object </p>
   * A cardinal rectangular region defined by two corners
   */
  public class Box extends GeoFence
  {
    private double Xa;
    private double Ya;
    private double Xb;
    private double Yb;

    public Box(double Xa, double Ya, double Xb, double Yb, double radius, double buffer)
    {
      this.Xa = Xa;
      this.Ya = Ya;
      this.Xb = Xb;
      this.Yb = Yb;

      this.radius = radius;
      this.buffer = buffer;

      centre = new Translation2d((Xa + Xb)/2, (Ya + Yb)/2);
    }

    public Box(double Xa, double Ya, double Xb, double Yb)
    {
      this(Xa, Ya, Xb, Yb, minRadius, minBuffer);
    }

    @Override
    public double getDistance()
    {
      double distance;
      if (robotPos.getX() < Xa)
        {
          if (robotPos.getY() < Ya) // SW Corner
            {distance = robotPos.getDistance(new Translation2d(Xa, Ya));}
          else if (robotPos.getY() > Yb) // NW Corner
            {distance = robotPos.getDistance(new Translation2d(Xa, Yb));}
          else // W Cardinal
            {distance = (Xa - radius) - robotPos.getX();}
        }
        else if (robotPos.getX() > Xb)
        {
          if (robotPos.getY() < Ya) // SE Corner
            {distance = robotPos.getDistance(new Translation2d(Xb, Ya));}
          else if (robotPos.getY() > Yb) // NE Corner
            {distance = robotPos.getDistance(new Translation2d(Xb, Yb));}
          else // E Cardinal
            {distance = robotPos.getX() - (Xb + radius);}
        }
        else 
        {
          if (robotPos.getY() < Ya) // S Cardinal
            {distance = (Ya - radius) - robotPos.getY();} 
          else if (robotPos.getY() > Yb) // N Cardinal
            {distance = robotPos.getY() - (Yb + radius);}
          else // Center (you've met a terrible fate *insert kazoo music here*)
            {distance = 0;}
        }
      return Math.abs(distance);
    }

    @Override
    protected boolean checkPosition()
    {
      return 
      !(
        (robotPos.getX() <= Xa - (radius + buffer + robotRadius)) || // Far from -X barrier
        (robotPos.getX() >= Xb + (radius + buffer + robotRadius)) || // Far from +X barrier
        (robotPos.getY() <= Ya - (radius + buffer + robotRadius)) || // Far from -Y barrier
        (robotPos.getY() >= Yb + (radius + buffer + robotRadius))    // Far from +Y barrier
      );
    }

    @Override
    protected Translation2d dampMotion(Translation2d motionXY)
    {
      double motionX = motionXY.getX();
      double motionY = motionXY.getY();
      double distanceToEdgeX;
      double distanceToEdgeY;

      if (robotPos.getX() < Xa)
        {
          if (robotPos.getY() < Ya) // SW Corner
            {return pointDamping(Xa, Ya, motionXY);}
          else if (robotPos.getY() > Yb) // NW Corner
            {return pointDamping(Xa, Yb, motionXY);}
          else // W Cardinal
          {
            distanceToEdgeX = (Xa - radius) - (robotPos.getX() + robotRadius);
            motionX = Math.min(motionX, (Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
          }
        }
        else if (robotPos.getX() > Xb)
        {
          if (robotPos.getY() < Ya) // SE Corner
            {return pointDamping(Xb, Ya, motionXY);}
          else if (robotPos.getY() > Yb) // NE Corner
            {return pointDamping(Xb, Yb, motionXY);}
          else // E Cardinal
          {
            distanceToEdgeX = (robotPos.getX() - robotRadius) - (Xb + radius);
            motionX = Math.max(motionX, (-Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
          }
        }
        else 
        {
          if (robotPos.getY() < Ya) // S Cardinal
          {
            distanceToEdgeY = (Ya - radius) - (robotPos.getY() + robotRadius);
            motionY = Math.min(motionY, (Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
          } 
          else if (robotPos.getY() > Yb) // N Cardinal
          {
            distanceToEdgeY = (robotPos.getY() - robotRadius) - (Yb + radius);
            motionY = Math.max(motionY, (-Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
          }
          else // Center (you've met a terrible fate *insert kazoo music here*)
            {return pointDamping(centre.getX(), centre.getY(), motionXY);}
        }
        return new Translation2d(motionX, motionY);
    }
  }

  /**
   * Line type GeoFence object </p>
   * Defined between two points </p>
   * Note: causes edge-case behaviours when meeting other objects at acute angles
   */
  public class Line extends GeoFence
  {
    private double Xa;
    private double Ya;
    private double Xb;
    private double Yb;
    private double dXab = 0;
    private double dYab = 0;
    private double dot2ab = 0;
    private double checkRadius;

    public Line(double Xa, double Ya, double Xb, double Yb, double radius, double buffer)
    {
      this.Xa = Xa;
      this.Ya = Ya;
      this.Xb = Xb;
      this.Yb = Yb;

      this.radius = radius;
      this.buffer = buffer;

      centre = new Translation2d((Xa+Xb)/2, (Ya+Yb)/2);

      dXab = Xb - Xa;
      dYab = Yb - Ya;
      dot2ab = Math.pow(dXab,2) + Math.pow(dYab,2);

      checkRadius = (Math.sqrt(dot2ab)/2) + radius;
    }

    public Line(double Xa, double Ya, double Xb, double Yb)
    {
      this(Xa, Ya, Xb, Yb, minRadius, minBuffer);
    }

    @Override
    protected boolean checkPosition()
    {
      return centre.getDistance(robotPos) <= checkRadius + buffer + robotRadius;
    }

    @Override
    protected Translation2d dampMotion(Translation2d motionXY)
    {
      /*
      * Calculates the nearest point on the line to the robot
      * Uses the dot product of the lines A-B and A-Robot to project the robot position onto the line
      * Then clamps the calculated point between the line endpoints
      *      
      *            /              (robotX - aX) * (bX - aX) + (robotY - aY) * (bY - aY) \
      *      aXY + | (bXY - aXY) *   ------------------------------------------------   |
      *            \                            (bX - aX)^2 + (bY - aY)^2               /
      */

      double distanceToEdgeX = robotPos.getX() - Xa;
      double distanceToEdgeY = robotPos.getY() - Ya;
      double dot = ((distanceToEdgeX * dXab) + (distanceToEdgeY * dYab)) / dot2ab; // Normalised dot product of the two lines
      return pointDamping
      (
        Conversions.clamp(Xa + dXab * dot, Xa, Xb), 
        Conversions.clamp(Ya + dYab * dot, Ya, Yb), 
        motionXY
      );
    }

    @Override
    public double getDistance()
    {
      double distanceToEdgeX = robotPos.getX() - Xa;
      double distanceToEdgeY = robotPos.getY() - Ya;
      double dot = ((distanceToEdgeX * dXab) + (distanceToEdgeY * dYab)) / dot2ab; // Normalised dot product of the two lines
      return new Translation2d
      (
        Conversions.clamp(Xa + dXab * dot, Xa, Xb), 
        Conversions.clamp(Ya + dYab * dot, Ya, Yb)
      )
      .getDistance(robotPos) - (radius + robotRadius);
    }
  }

  /**
   * Polygon type GeoFence object </p>
   * A rotated regular polygon built from a series of Line objects </p>
   * with handling to only process the nearest line
   */
  public class Polygon extends GeoFence
  {
    List<Line> edgeLines;
    List<Translation2d> edgeReference;
    int sides;

    /**
      * Define regular polygon object
      * @param X x-coordinate of centre, metres
      * @param Y y-coordinate of centre, metres
      * @param buffer range over which the robot slows down, metres  
      * @param radius circumscribed (centre-corner) radius of the object, metres
      * @param theta angle of the object: 0 = "corner at North", degrees Anticlockwise
      * @param sides number of polygon sides, integer [3..12]
      */
    public Polygon(double X, double Y, double radius, double buffer, double theta, int sides)
    {
      edgeLines = new ArrayList<Line>();
      edgeReference = new ArrayList<Translation2d>();

      centre = new Translation2d(X,Y);

      // Constraining inputs
      radius = Math.max(Math.abs(radius), minRadius);
      buffer = Math.max(buffer, minBuffer);
      sides = Conversions.clamp(sides, 3, 12);
      
      // Array of all points to construct the polygon lines and references
      // The start of the first line and end of the last line are separate enteries to simplify construction
      Translation2d[] polygonPoints = new Translation2d[2*sides+1];

      polygonPoints[0] = new Translation2d(X,Y + radius).rotateAround(centre, new Rotation2d(Units.degreesToRadians(theta)));

      // Line endpoints and reference points are equidistant around a circle
      Rotation2d rotationBetweenPoints = new Rotation2d(Units.degreesToRadians(360/(2*sides)));
      for (int i = 1; i < polygonPoints.length; i++)
        {polygonPoints[i] = polygonPoints[i-1].rotateAround(centre, rotationBetweenPoints);}

      for (int i = 0; i < sides; i++)
      {
        edgeLines.add(i, new Line
        (
          polygonPoints[2*i].getX(),
          polygonPoints[2*i].getY(),
          polygonPoints[2*i+2].getX(),
          polygonPoints[2*i+2].getY(),
          0,
          buffer
        ));

        edgeReference.add(i, polygonPoints[2*i+1]);
      }

      /* 
        * Convert the circumscribed radius (centre-corner) to the inscribed radius (centre-edge)
        * and expand the buffer to account for the difference
        * 
        * These values are used to process the polygon as a point if the robot crosses the lines
        */ 
      this.radius = rotationBetweenPoints.getCos() * radius;
      this.buffer = buffer + (radius - this.radius);
    }

    @Override
    protected Translation2d dampMotion(Translation2d motionXY)
    {
      // If the robot is touching (or past) the inscribed circle, process based on that circle
      if (centre.getDistance(robotPos) <= radius)
        {return pointDamping(centre.getX(), centre.getY(), motionXY);}
      else 
      {
        /* 
          * Damps the motion based on the line closest to the robot:
          * Polygon objects consist of a list of lines and a list of reference points
          * Finding the index of the closest reference point gives the index of the closest line
          */
        return edgeLines.get(edgeReference.indexOf(robotPos.nearest(edgeReference))).dampMotion(motionXY);
      }
    }

    @Override
    public double getDistance()
    {
      return edgeLines.get(edgeReference.indexOf(robotPos.nearest(edgeReference))).getDistance();
    }
  }
}
