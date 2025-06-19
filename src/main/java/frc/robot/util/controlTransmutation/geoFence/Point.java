// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation.geoFence;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.controlTransmutation.GeoFence;

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

    checkRadius = radius + buffer;
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