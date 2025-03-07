// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.FieldUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DynamicBargeObstacle extends Command 
{
  private Supplier<Translation2d> robotPosSup;
  private ArrayList<Pair<Translation2d, Translation2d>> bargeObstacle;

  public DynamicBargeObstacle(Supplier<Translation2d> robotPosSup) 
  {
    this.robotPosSup = robotPosSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    bargeObstacle.add(FieldUtils.isRedAlliance() ? FieldUtils.GeoFencing.redAllianceBargeDynamic : FieldUtils.GeoFencing.blueAllianceBargeDynamic);

    Pathfinding.setDynamicObstacles(bargeObstacle, robotPosSup.get());  
  }

  @Override
  public void end(boolean interrupted) 
  {
    Pathfinding.setDynamicObstacles(new ArrayList<Pair<Translation2d, Translation2d>>(), robotPosSup.get()); 
  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
