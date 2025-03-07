  // Copyright (c) FIRST and other WPILib contributors./ Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Diffector.GoToCoralScorePos;
import frc.robot.commands.Manipulator.SetCoralStatus;
import frc.robot.commands.Util.WaitUntilAutoTime;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Auto.AutoMapping;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;

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
    // Removes all space characters from the single-String command phrases, ensures it's all lowercase, and then splits it into individual strings, which are stored in an array
    String[] splitCommands = commandInput.replaceAll("//s", "").toLowerCase().split(",");
    // The arraylist that all the commands will be placed into
    ArrayList<Command> commandList = new ArrayList<>();
    
    // Holders for the values during each command processing
    AutoMapping autoMapValue;
    PathPlannerPath nextPath;
    Command command;

    // Tracks the end point of the previous path, used so each path properly pathfinds from the end point of the previous one
    Translation2d prevEndPoint = RobotContainer.swerveState.Pose.getTranslation();

    // For each command phrase, adds the associated path and then the associated command to the command list
    for (int i = 0; i < splitCommands.length; i++) 
    {
        // 'w' is a command
        if (splitCommands[i].charAt(0) == 'w')
          {commandList.add(new WaitCommand(Double.parseDouble(splitCommands[i].substring(1))));}
        // 't' is a wait until match time command
        else if (splitCommands[i].charAt(0) == 't') 
          {commandList.add(new WaitUntilAutoTime(Double.parseDouble(splitCommands[i].substring(1))));}
        else if (splitCommands[i].charAt(0) == 'r')
        {
          nextPath = FieldUtils.loadPath(Constants.Auto.autoMap.get(splitCommands[i].substring(0, 2)).pathName);

          Pathfinding.setStartPosition(prevEndPoint);
          
          commandList.add
            (
              AutoBuilder.pathfindThenFollowPath(nextPath, constraints)
              .alongWith
              (
                new GoToCoralScorePos
                (
                  Integer.parseInt(splitCommands[i].substring(2)), 
                  RobotContainer.s_Diffector, 
                  () -> RobotContainer.swerveState.Pose.getTranslation()
                )
              )
            );
          
          prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();

          commandList.add(new SetCoralStatus(RobotContainer.s_CoralManipulator, CoralManipulatorStatus.DELIVERY_SMART));
        }
        else
        {
          // Each iteration fills two indexes in the command list
          autoMapValue = Constants.Auto.autoMap.get(splitCommands[i]);
          nextPath = FieldUtils.loadPath(autoMapValue.pathName);

          Pathfinding.setStartPosition(prevEndPoint);
          
          commandList.add(AutoBuilder.pathfindThenFollowPath(nextPath, constraints));
          prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();    

          command = autoMapValue.command.get();

          if (command != null) 
          {
            commandList.add(command);
          }
        }
    }

    return commandList;
  }
}
