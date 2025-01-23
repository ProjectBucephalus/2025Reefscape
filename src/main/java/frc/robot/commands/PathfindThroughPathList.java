// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;

public class PathfindThroughPathList extends SequentialCommandGroup 
{
  private final PathConstraints constraints = Constants.Auto.defaultConstraints;

  public PathfindThroughPathList(PathPlannerPath[] pathList) 
  {
    Command[] commandList = new Command[pathList.length];

    for (int i = 0; i < commandList.length; i++) 
    {
      commandList[i] = AutoBuilder.pathfindThenFollowPath(pathList[i], constraints);
    }

    addCommands(commandList);
  }
}
