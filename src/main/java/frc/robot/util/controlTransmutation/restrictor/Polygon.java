// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation.restrictor;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;
import frc.robot.util.controlTransmutation.Restrictor;

/** Add your docs here. */
public class Polygon extends Restrictor 
{
  private ArrayList<Line> polygonLines;
  
  public Polygon(double X, double Y, double radius, double buffer, double theta, int sides, double localSpeedLimit)
  {
    polygonLines = new ArrayList<Line>();

    centre = new Translation2d(X,Y);

    // Constraining inputs
    radius = Math.max(Math.abs(radius), minRadius);
    buffer = Math.max(buffer, minBuffer);
    sides = Conversions.clamp(sides, 3, 12);
    
    // Array of all points to construct the polygon lines and references
    // The start of the first line and end of the last line are separate enteries to simplify construction
    Translation2d[] polygonPoints = new Translation2d[sides+1];

    polygonPoints[0] = new Translation2d(X,Y + radius).rotateAround(centre, Rotation2d.fromDegrees(theta));

    // Line endpoints are equidistant around a circle
    Rotation2d rotationBetweenPoints = Rotation2d.fromDegrees(360/sides);
    for (int i = 1; i < polygonPoints.length; i++)
      {polygonPoints[i] = polygonPoints[i-1].rotateAround(centre, rotationBetweenPoints);}

    for (int i = 0; i < sides; i++)
    {
      polygonLines.add
      (i, new Line
        (
          polygonPoints[i],
          polygonPoints[i+1]
        )
      );
    }

    /* 
      * Convert the circumscribed radius (centre-corner) to the inscribed radius (centre-edge)
      * and expand the buffer to account for the difference
      * 
      * These values are used to process the polygon as a point if the robot crosses the lines
      */ 
    this.radius = rotationBetweenPoints.getCos() * radius;
    this.buffer = buffer + (radius - this.radius);

    checkRadius = radius + buffer;
  }

  public double getDistance()
  {
    return nearestLine().getDirectionalDistance();
  }

  private Line nearestLine()
  {
    int index = 0;
    double minDistance = polygonLines.get(0).getCentre().getDistance(robotPos);
    double checkDistance;

    for (int i = 1; i < polygonLines.size(); i++)
    {
      checkDistance = polygonLines.get(i).getCentre().getDistance(robotPos);
      if (checkDistance < minDistance)
      {
        index = i;
        minDistance = checkDistance;
      }
    }

    return polygonLines.get(index);
  }
}
