package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeoFenceObject
{
    /*
     * NOTE: +X is robot Forward, called North, +Y is robot Left, called West
     */

    public enum ObjectTypes {walls, box, line, point, polygon};

    private double Xa;
    private double Ya;
    private double Xb;
    private double Yb;
    private ObjectTypes objectType;
    private double buffer;
    private double radius;
    private Translation2d centre;

    // Specialised values for line objects
    private double dXab = 0;
    private double dYab = 0;
    private double dot2ab = 0;

    // Specialised values for polygon objects
    List<GeoFenceObject> edgeLines;
    List<Translation2d> edgeReference; 


    /**
     * Default empty constructor
     */
    public GeoFenceObject()
    {
        this(-100,-100);
    }

    /**
     * Define a minimum point-type object to avoid
     * @param x x-coordinate, metres
     * @param y y-coordinate, metres
     */
    public GeoFenceObject(double x, double y)
    {
        this(x, y, 0.5, 0);
    }

    /**
     * Define a point-type object to avoid
     * @param x x-coordinate, metres
     * @param y y-coordinate, metres
     * @param buffer range over which the robot slows down, metres
     * @param radius hard-stop radius around object, metres
     */
    public GeoFenceObject(double x, double y, double buffer, double radius)
    {
        this(x, y, x, y, buffer, radius, ObjectTypes.point);
    }

    /**
     * Define a line-type object to avoid 
     * @param Xa x-coordinate in meters of the first point
     * @param Ya y-coordinate in meters of the first point
     * @param Xb x-coordinate in meters of the second point
     * @param Yb y-coordinate in meters of the second point
     * @param buffer range over which the robot slows down, metres  
     */
    public GeoFenceObject(double Xa, double Ya, double Xb, double Yb, double buffer) 
    {
        this(Xa, Ya, Xb, Yb, buffer, 0, ObjectTypes.line);
    }

    /**
     * Define fully custom geofence
     * @param Xa Start x-coordinate of region, metres
     * @param Ya Start y-coordinate of region, metres
     * @param Xb Size of region in x-axis, metres
     * @param Yb Size of region in y-axis, metres
     * @param buffer range over which the robot slows down, metres  
     * @param radius hard-stop radius around object, metres
     * @param objectType Type of object to avoid. Walls (stay within area), box (stay outside area), line, or point
     */
    public GeoFenceObject(double Xa, double Ya, double Xb, double Yb, double buffer, double radius, ObjectTypes objectType)
    {
        if (objectType == ObjectTypes.walls || objectType == ObjectTypes.box)
        {
            this.Xa = Math.min(Xa, Xb);
            this.Ya = Math.min(Ya, Yb);
            this.Xb = Math.max(Xa, Xb);
            this.Yb = Math.max(Ya, Yb);
            centre = new Translation2d((Xa+Xb)/2, (Ya+Yb)/2);
        }
        else if (objectType == ObjectTypes.point)
        {
            this.Xa = Xa;
            this.Xb = Xa;
            this.Ya = Ya;
            this.Yb = Ya;
            centre = new Translation2d(Xa, Ya);
        }
        else
        {
            this.Xa = Xa;
            this.Xb = Xb;
            this.Ya = Ya;
            this.Yb = Yb;

            centre = new Translation2d((Xa+Xb)/2, (Ya+Yb)/2);

            this.dXab = Xb - Xa;
            this.dYab = Yb - Ya;
            this.dot2ab = Math.pow(dXab,2) + Math.pow(dYab,2);
        }
        this.objectType = objectType;
        this.buffer = Math.max(buffer, 0.1);
        this.radius = radius;
    }

    /**
     * Define regular polygon object
     * @param X x-coordinate of centre, metres
     * @param Y y-coordinate of centre, metres
     * @param buffer range over which the robot slows down, metres  
     * @param radius circumscribed (centre-corner) radius of the object, metres
     * @param theta angle of the object: 0 = "corner at North", degrees Anticlockwise
     * @param sides number of polygon sides, integer [3..12]
     */
    public GeoFenceObject(double X, double Y, double buffer, double radius, double theta, int sides)
    {
        edgeLines = new ArrayList<GeoFenceObject>();
        edgeReference = new ArrayList<Translation2d>();

        Xa = X;
        Ya = Y;
        Xb = X;
        Yb = Y;
        centre = new Translation2d(X,Y);

        // Constraining inputs
        buffer = Math.max(buffer, 0.1);
        radius = Math.abs(radius);
        objectType = ObjectTypes.polygon;
        sides = Conversions.clamp(sides,3,12);
        
        // Array of all points to construct the polygon lines and references
        // The start of the first line and end of the last line are separate enteries to simplify construction
        Translation2d[] polygonPoints = new Translation2d[2*sides+1];

        polygonPoints[0] = new Translation2d(X,Y + radius).rotateAround(centre, new Rotation2d(Math.toRadians(theta)));

        // Line endpoints and reference points are equidistant around a circle
        Rotation2d rotationBetweenPoints = new Rotation2d(Math.toRadians(360/(2*sides)));
        for (int i = 1; i < polygonPoints.length; i++)
        {
            polygonPoints[i] = polygonPoints[i-1].rotateAround(centre, rotationBetweenPoints);
        }

        for (int i = 0; i < sides; i++)
        {
            edgeLines.add(i, new GeoFenceObject
            (
                polygonPoints[2*i].getX(),
                polygonPoints[2*i].getY(),
                polygonPoints[2*i+2].getX(),
                polygonPoints[2*i+2].getY(),
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

    /**
     * Determines if the robot is close enough to the Geofence object to potentially need motion damping
     * @param robotXY Coordinates of the robot, metres
     * @param checkRadius Combined radius of Object, Robot, and Buffer, metres
     * @return BOOLEAN is the robot close enough to the object to check more thoroughly
     */
    public boolean checkPosition(Translation2d robotXY, double checkRadius)
    {
        // det. robotXY within (Geofence + robotR + buffer) box
        if (objectType == ObjectTypes.walls)
        {
            return
            (
                (robotXY.getX() >= Xb - checkRadius) ||  // Close to inside of +X barrier
                (robotXY.getX() <= Xa + checkRadius) ||  // Close to inside of -X barrier
                (robotXY.getY() >= Yb - checkRadius) ||  // Close to inside of +Y barrier
                (robotXY.getY() <= Ya + checkRadius)     // Close to inside of -Y barrier
            );
        }
        else if (objectType == ObjectTypes.box)
        {
            return
            !(
                (robotXY.getX() <= Xa - checkRadius) || // Far from -X barrier
                (robotXY.getX() >= Xb + checkRadius) || // Far from +X barrier
                (robotXY.getY() <= Ya - checkRadius) || // Far from -Y barrier
                (robotXY.getY() >= Yb + checkRadius)    // Far from +Y barrier
            );
        }
        else
        {
            return
            !(
                (robotXY.getX() <= Math.min(Xa, Xb) - checkRadius) || // Far from -X barrier
                (robotXY.getX() >= Math.max(Xa, Xb) + checkRadius) || // Far from +X barrier
                (robotXY.getY() <= Math.min(Ya, Yb) - checkRadius) || // Far from -Y barrier
                (robotXY.getY() >= Math.max(Ya, Yb) + checkRadius)    // Far from +Y barrier
            );
        }
    }

    private Translation2d pointDamping(Translation2d point, Translation2d motionXY, double robotR, Translation2d robotXY)
        {return pointDamping(point.getX(), point.getY(), motionXY, robotR, robotXY);}
    
    private Translation2d pointDamping(double pointX, double pointY, Translation2d motionXY, double robotR, Translation2d robotXY)
    {
        // Calculates X and Y distances to the point
        double distanceX = pointX - robotXY.getX();
        double distanceY = pointY - robotXY.getY();
        // Calculates the normal distance to the corner through pythagoras; this is the actual distance between the robot and point
        double distanceN = Math.hypot(distanceX, distanceY);
        // Calculates the robot's motion normal and tangent to the point; i.e., towards and away from the point, and from side to side relative to the point
        double motionN   = ((distanceX * motionXY.getX()) + (distanceY * motionXY.getY())) / distanceN;
        double motionT   = ((distanceX * motionXY.getY()) - (distanceY * motionXY.getX())) / distanceN;
        
        // Clamps the normal motion, i.e. motion towards the point, in order to clamp robot speed
        // Sets maximum input towards the object as:
        //      (position within the buffer normalised to [0..1])   *   (angle normalisation factor [1..sqrt(2)])
        //         (dNormal - object radii)[0..buffer] / buffer     *      (mNormal / max(|X|,|Y|))
        motionN = Math.min(motionN, motionN * Conversions.clamp(distanceN-(robotR + radius), 0, buffer)
                                        / (Math.max(Math.abs(distanceX),Math.abs(distanceY)) * buffer));
        
        // Converts clamped motion from normal back to X and Y
        double motionX   = ((motionN * distanceX) - (motionT * distanceY)) / distanceN;
        double motionY   = ((motionN * distanceY) + (motionT * distanceX)) / distanceN;

        return new Translation2d(motionX, motionY);
    }

    /**
     * Damps the motion of the robot in the direction of a Geofence object to prevent collision
     * @param robotXY Coordinates of the robot, metres
     * @param motionXY Control input to be damped, must be aligned to field coordinates
     * @param robotR Effective radius of the robot, metres
     * @return Modified input to send to drive command
     */
    public Translation2d dampMotion(Translation2d robotXY, Translation2d motionXY, double robotR)
    {
        double distanceToEdgeX;
        double distanceToEdgeY;
        
        if (!checkPosition(robotXY, robotR + radius + buffer)) {return motionXY;}
        // orth. v. diagonal v. internal
        double motionX = motionXY.getX();
        double motionY = motionXY.getY();
        
        //if(objectType != null)
        switch (objectType) 
        {
            case point:
                return pointDamping(centre, motionXY, robotR, robotXY);

            case line:
                   /*
                    * Calculates the nearest point on the line to the robot
                    * Uses the dot product of the lines A-B and A-Robot to project the robot position onto the line
                    * Then clamps the calculated point between the line endpoints
                    *      
                    *            /              (robotX - aX) * (bX - aX) + (robotY - aY) * (bY - aY) \
                    *      aXY + | (bXY - aXY) *   ------------------------------------------------   |
                    *            \                            (bX - aX)^2 + (bY - aY)^2               /
                    */
                distanceToEdgeX = robotXY.getX() - Xa;
                distanceToEdgeY = robotXY.getY() - Ya;
                double dot = ((distanceToEdgeX * dXab) + (distanceToEdgeY * dYab)) / dot2ab; // Normalised dot product of the two lines
                return pointDamping(Conversions.clamp(Xa + dXab * dot, Xa, Xb), Conversions.clamp(Ya + dYab * dot, Ya, Yb), motionXY, robotR, robotXY);

            case box:
                if (robotXY.getX() < Xa)
                {
                    if (robotXY.getY() < Ya) // SW Corner
                    {
                        return pointDamping(Xa, Ya, motionXY, robotR, robotXY);
                    }
                    else if (robotXY.getY() > Yb) // NW Corner
                    {
                        return pointDamping(Xa, Yb, motionXY, robotR, robotXY);
                    }
                    else // W Cardinal
                    {
                        distanceToEdgeX = Math.abs((Xa - radius) - (robotXY.getX() + robotR)); 
                        motionX = Math.min(motionX, (Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
                    }
                }
                else if (robotXY.getX() > Xb)
                {
                    if (robotXY.getY() < Ya) // SE Corner
                    {
                        return pointDamping(Xb, Ya, motionXY, robotR, robotXY);
                    }
                    else if (robotXY.getY() > Yb) // NE Corner
                    {   
                        return pointDamping(Xb, Yb, motionXY, robotR, robotXY);
                    }
                    else // E Cardinal
                    {
                        distanceToEdgeX = Math.abs((robotXY.getX() - robotR) - (Xb + radius));
                        motionX = Math.max(motionX, (-Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
                    }
                }
                else 
                {
                    if (robotXY.getY() < Ya) // S Cardinal
                    {
                        distanceToEdgeY = Math.abs((Ya - radius) - (robotXY.getY() + robotR));
                        motionY = Math.min(motionY, (Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
                    } 
                    else if (robotXY.getY() > Yb) // N Cardinal
                    {
                        distanceToEdgeY = Math.abs((robotXY.getY() - robotR) - (Yb + radius));
                        motionY = Math.max(motionY, (-Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
                    }
                    else // Center (you've met a terrible fate *insert kazoo music here*)
                    {
                        return pointDamping(centre, motionXY, robotR, robotXY);                                  
                    } 
                }
                return new Translation2d(motionX, motionY);

            case walls:
                // Calculates distance to the relevant edge of the field
                // Calculates edge position, and subtracts robot position + radius from edge position.

                // Sets the motion in the relevant direction to the minimum of the current motion
                // And the distance from the edge clamped between 0 and the edge buffer, and normalised to a maximum of 1.
                // This ensures the motion in that direction does not go above the clamped + normalised distance from the edge, to cap speed.
                if (motionX > 0)
                {   
                    distanceToEdgeX = (Xb - radius) - (robotXY.getX() + robotR); 
                    motionX = Math.min(motionX, (Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
                }
                else if (motionX < 0)
                {   
                    distanceToEdgeX = (robotXY.getX() - robotR) - (Xa + radius);
                    motionX = Math.max(motionX, (-Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
                }

                if (motionY > 0)
                {   
                    distanceToEdgeY = (Yb - radius) - (robotXY.getY() + robotR);
                    motionY = Math.min(motionY, (Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
                }
                else if (motionY < 0)
                {   
                    distanceToEdgeY = (robotXY.getY() - robotR) - (Ya + radius);
                    motionY = Math.max(motionY, (-Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
                }
                return new Translation2d(motionX, motionY);
            
            case polygon:
                // If the robot is touching (or past) the inscribed circle, process based on that circle
                if (robotXY.getDistance(centre) <= radius)
                    {return pointDamping(centre, motionXY, robotR, robotXY);}
                else 
                {
                    /* 
                     * Damps the motion based on the line closest to the robot:
                     * Polygon objects consist of a list of lines and a list of reference points
                     * Finding the index of the closest reference point gives the index of the closest line
                     */
                    return edgeLines.get(edgeReference.indexOf(robotXY.nearest(edgeReference))).dampMotion(robotXY, motionXY, robotR);
                }
            
            default:
                return motionXY;
        }            
    }
}