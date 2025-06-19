// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation.restrictor;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.controlTransmutation.Restrictor;

/** Add your docs here. */
public class Polygon extends Restrictor 
{
  private class Line
  {
    private Translation2d pointA;
    private Translation2d pointB;
    private Translation2d centre;
    

    private double dXab;
    private double dYab;
    private double length;
    private double normX;
    private double normY;
    private double normXY;

    private double dotX;
    private double dotY;
    private double dotXY;
    

    private Line(Translation2d pointA, Translation2d pointB)
    {
      this.pointA = pointA;
      this.pointB = pointB;
      centre = new Translation2d((pointA.getX() + pointB.getX())/2, (pointA.getY() + pointB.getY())/2);
      
      dXab = pointB.getX() - pointA.getX();
      dYab = pointB.getY() - pointA.getY();
      length = Math.hypot(dXab, dYab);
      normX = dXab / length;
      normY = dYab / length;
      normXY = ((pointA.getX() * pointB.getY()) - (pointB.getX() * pointA.getY())) / length;

      dotX = normX / length;
      dotY = normY / length;
      dotXY = pointA.getX()*dXab + pointA.getY()*dYab;
    }

    private double getDistance()
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

  private ArrayList<Line> polygonLines;
  
  public Polygon()
  {
    
  }
}
