// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation.geoFence;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;
import frc.robot.util.controlTransmutation.GeoFence;

/**
 * Line type GeoFence object </p>
 * Defined between two points </p>
 * Note: causes edge-case behaviours when meeting other objects at acute angles
 */
public class Line extends GeoFence
{
  private Translation2d pointA;
  private Translation2d pointB;
  private double dXab;
  private double dYab;
  private double length;
  private double dotX;
  private double dotY;
  private double dotXY;
  private double normX;
  private double normY;
  private double normXY;

  public Line(double Xa, double Ya, double Xb, double Yb, double radius, double buffer)
  {
    this.radius = radius;
    this.buffer = buffer;
    
    pointA = new Translation2d(Xa, Ya);
    pointB = new Translation2d(Xb, Yb);
    centre = new Translation2d((Xa+Xb)/2, (Ya+Yb)/2);

    dXab = Xb - Xa;
    dYab = Yb - Ya;
    length = Math.hypot(dXab, dYab);
    
    normX = dXab / length;
    normY = dYab / length;
    normXY = ((Xa * Yb) - (Xb * Ya)) / length;
    
    dotX = normX / length;
    dotY = normY / length;
    dotXY = Xa*dXab + Ya*dYab;

    checkRadius = (Math.sqrt(length)/2) + radius + buffer;
  }

  public Line(double Xa, double Ya, double Xb, double Yb)
  {
    this(Xa, Ya, Xb, Yb, minRadius, minBuffer);
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

    double dot = (robotPos.getX() * dotX) + (robotPos.getY() * dotY) - dotXY; // Normalised dot product of the two lines
    return pointDamping
    (
      Conversions.clamp(pointA.getX() + dXab * dot, pointA.getX(), pointB.getX()), 
      Conversions.clamp(pointA.getY() + dYab * dot, pointA.getY(), pointB.getY()), 
      motionXY
    );
  }

  @Override
  public double getDistance()
  {
    double dot = (robotPos.getX() * dotX) + (robotPos.getY() * dotY) - dotXY; // Normalised dot product of the two lines
    return new Translation2d
    (
      Conversions.clamp(pointA.getX() + dXab * dot, pointA.getX(), pointB.getX()), 
      Conversions.clamp(pointA.getY() + dYab * dot, pointA.getY(), pointB.getY())
    )
    .getDistance(robotPos) - (radius + robotRadius);
  }

  /** If the robot position is within the projection area of the line, the output will be negative on one side of the line */
  public double getDirectionalDistance()
  {
    double dot = (robotPos.getX() * dotX) + (robotPos.getY() * dotY) - dotXY;
    if (dot <= 0)
      {return pointA.getDistance(robotPos);}
    else if (dot >= 1)
      {return pointB.getDistance(robotPos);}
    else
      {return ((robotPos.getX() * normY) + (robotPos.getY() * normX) + normXY);}
  }
}