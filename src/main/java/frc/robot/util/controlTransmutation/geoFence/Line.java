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