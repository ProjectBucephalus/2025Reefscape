// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class DynamicAuto {
  /** Creates a new DynamicAuto. */
  public DynamicAuto() {}

  /**
   * Splits a string of auto commands and gets the path associated with each command
   * @param commandInput The string of commands to split, seperated by commas with no spaces (e.g. "a1,rA1,p,cR3")
   * @return An array of path names, from the input command string, in the same order
   */
  public static PathPlannerPath[] getPathList(String commandInput)
  {
    String[] splitCommands = commandInput.split(",");
    PathPlannerPath[] pathList = new PathPlannerPath[splitCommands.length];
  
    for (int i = 0; i < pathList.length; i++) 
    {
      try
      {
        pathList[i] = PathPlannerPath.fromPathFile(Constants.Auto.pathMap.get(splitCommands[i]));
      } 
      catch (Exception e) 
      {
          DriverStation.reportError("Path error: " + e.getMessage(), e.getStackTrace());
      }  
    }

    return pathList;
  }
}
