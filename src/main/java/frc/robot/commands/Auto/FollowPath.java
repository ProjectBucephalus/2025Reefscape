// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.FieldUtils;

public class FollowPath extends Command 
{
  private PathPlannerPath path;

  public FollowPath(String pathName) 
    {path = FieldUtils.loadPath(pathName);}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
    {AutoBuilder.followPath(path).schedule();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return true;}
}
