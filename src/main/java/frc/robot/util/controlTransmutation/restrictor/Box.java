// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation.restrictor;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.controlTransmutation.Restrictor;

/** Add your docs here. */
public class Box extends Restrictor 
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
}
