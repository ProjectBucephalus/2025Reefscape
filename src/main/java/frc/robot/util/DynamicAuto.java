// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Auto.AutoMapping;

public class DynamicAuto 
{
  private static final PathConstraints constraints = Constants.Auto.defaultConstraints;
  
  /** Creates a new DynamicAuto. */
  public DynamicAuto() {}

  /**
   * Splits a string of auto command phrases and gets the path command and robot command associated with each command phrase
   * @param commandInput The string of commands to split, seperated by commas with no spaces (e.g. "a1,rA1,p,cR3")
   * @return An array of commands, from the input command phrase string, in the same order
   */
  public static ArrayList<Command> getCommandList(String commandInput)
  {
    // Splits the single-String command phrases into individual strings, which are stored in an array
    String[] splitCommands = commandInput.toLowerCase().split(",");
    // Length of the command list is twice the number of command phrases, as each phrase maps to two commands (one path and one robot)
    ArrayList<Command> commandList = new ArrayList<>();
    
    AutoMapping autoMapValue;

    // For each command phrase, adds the associated path and then the associated command to the command list
    for (int i = 0; i < splitCommands.length; i++) 
    {
      try
      {
        // 'w' is a command
        if (splitCommands[i].charAt(0) == 'w')
        {
          commandList.add(new WaitCommand(Double.parseDouble(splitCommands[i].substring(1))));
        }
        // 't' is a wait until match time command
        else if (splitCommands[i].charAt(0) == 't') 
        {
          commandList.add(new WaitUntilCommand(Double.parseDouble(splitCommands[i].substring(1))));
        }
        else
        {
          // Each iteration fills two indexes in the command list
          autoMapValue = Constants.Auto.autoMap.get(splitCommands[i]);
          commandList.add(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(autoMapValue.pathName), constraints));
          commandList.add(autoMapValue.command);
        }
      } 
      catch (Exception e) 
      {
        DriverStation.reportError("Path error: " + e.getMessage(), e.getStackTrace());
      }  
    }

    return commandList;
  }
}
