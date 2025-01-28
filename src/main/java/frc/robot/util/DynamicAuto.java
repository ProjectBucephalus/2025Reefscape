// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralManipulator.ScoreCoral;
import frc.robot.commands.Util.WaitUntilAutoTime;
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
    Translation2d prevEndPoint = RobotContainer.s_Swerve.getState().Pose.getTranslation();
    PathPlannerPath nextPath;

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
          commandList.add(new WaitUntilAutoTime(Double.parseDouble(splitCommands[i].substring(1))));
        }
        else if (splitCommands[i].charAt(0) == 'r')
        {
          nextPath = PathPlannerPath.fromPathFile(Constants.Auto.autoMap.get(splitCommands[i].substring(0, 1)).pathName);

          Pathfinding.setStartPosition(prevEndPoint);
          
          commandList.add(AutoBuilder.pathfindThenFollowPath(nextPath, constraints));
          prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();

          commandList.add(new ScoreCoral(Integer.parseInt(splitCommands[i].substring(2)), RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator));
        }
        else
        {
          // Each iteration fills two indexes in the command list
          autoMapValue = Constants.Auto.autoMap.get(splitCommands[i]);
          nextPath = PathPlannerPath.fromPathFile(autoMapValue.pathName);

          Pathfinding.setStartPosition(prevEndPoint);
          
          commandList.add(AutoBuilder.pathfindThenFollowPath(nextPath, constraints));
          prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();    

          commandList.add(autoMapValue.command.get());
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
