// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

public class PathfindToAndFollow extends Command 
{
  private PathPlannerPath path;
  private final PathConstraints constraints = Constants.Auto.defaultConstraints;
  private Command pathfindingCommand;
  
  public PathfindToAndFollow(String pathName) 
  {
    try
    {
      path = PathPlannerPath.fromPathFile(pathName);
    } 
    catch (Exception e) 
    {
        DriverStation.reportError("Path error: " + e.getMessage(), e.getStackTrace());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    pathfindingCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
