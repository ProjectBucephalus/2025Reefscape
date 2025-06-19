// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation.geoFence;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;
import frc.robot.util.controlTransmutation.GeoFence;

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
    this.Xa = Math.min(Xa, Xb);
    this.Ya = Math.min(Ya, Yb);
    this.Xb = Math.max(Xa, Xb);
    this.Yb = Math.max(Ya, Yb);

    this.radius = radius;
    this.buffer = buffer;

    centre = new Translation2d((Xa + Xb)/2, (Ya + Yb)/2);

    checkRadius = radius + buffer;
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
      (robotPos.getX() >= Xb - (checkRadius + robotRadius)) ||  // Close to inside of +X barrier
      (robotPos.getX() <= Xa + (checkRadius + robotRadius)) ||  // Close to inside of -X barrier
      (robotPos.getY() >= Yb - (checkRadius + robotRadius)) ||  // Close to inside of +Y barrier
      (robotPos.getY() <= Ya + (checkRadius + robotRadius))     // Close to inside of -Y barrier
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