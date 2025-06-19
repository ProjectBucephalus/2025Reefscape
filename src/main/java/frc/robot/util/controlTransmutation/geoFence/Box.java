// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation.geoFence;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;
import frc.robot.util.controlTransmutation.GeoFence;

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
    this.Xa = Math.min(Xa, Xb);
    this.Ya = Math.min(Ya, Yb);
    this.Xb = Math.max(Xa, Xb);
    this.Yb = Math.max(Ya, Yb);

    this.radius = radius;
    this.buffer = buffer;

    centre = new Translation2d((Xa + Xb)/2, (Ya + Yb)/2);

    checkRadius = (Math.hypot(Xb - Xa, Yb - Ya)/2) + radius + buffer;
  }

  public Box(double Xa, double Ya, double Xb, double Yb)
  {
    this(Xa, Ya, Xb, Yb, minRadius, minBuffer);
  }

  @Override
  public double getDistance()
  {
    double distance = 0;

    if (robotPos.getX() < Xa)
    {
      if (robotPos.getY() < Ya)
        {distance = Math.hypot(Xa - robotPos.getX(), Ya - robotPos.getY());}
      else if (robotPos.getY() > Yb)
        {distance = Math.hypot(Xa - robotPos.getX(), robotPos.getY() - Yb);}
    }
    else if (robotPos.getX() > Xb)
    {
      if (robotPos.getY() < Ya)
        {distance = Math.hypot(robotPos.getX() - Xb, Ya - robotPos.getY());}
      else if (robotPos.getY() > Yb)
        {distance = Math.hypot(robotPos.getX() - Xb, robotPos.getY() - Yb);}
    }
    else
    {
      distance = Math.max
      (
        Math.max
        (
          Xa - robotPos.getX(), 
          Ya - robotPos.getY()
        ),
        Math.max
        (
          robotPos.getX() - Xb, 
          robotPos.getY() - Yb
        )
      );
    }

    return distance - (radius + robotRadius);
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