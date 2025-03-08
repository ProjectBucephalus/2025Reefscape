  // Copyright (c) FIRST and other WPILib contributors./ Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Diffector.GoToCoralScorePos;
import frc.robot.commands.Diffector.MoveTo;
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
  public static Command getCommandList(String commandInput)
  {
    // Removes all space characters from the single-String command phrases, ensures it's all lowercase, and then splits it into individual strings, which are stored in an array
    String[] splitCommands = commandInput.replaceAll("//s", "").toLowerCase().split(",");
    // The arraylist that all the commands will be placed into
    ArrayList<Command> commandList = new ArrayList<>();
    
    SmartDashboard.putStringArray("Split Commands", splitCommands);

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
        
        if (splitCommands[i].charAt(2) == '4') 
        {
          commandList.add(new WaitCommand(0.1));
          commandList.add(new GoToCoralScorePos(3, RobotContainer.s_Diffector, () -> RobotContainer.swerveState.Pose.getTranslation()));
        }
        
        commandList.add(new WaitUntilCommand(() -> !RobotContainer.coral));
        commandList.add(new SetCoralStatus(RobotContainer.s_CoralManipulator, CoralManipulatorStatus.DEFAULT));
      }
      else if (splitCommands[i].charAt(0) == 'c') 
      {
        nextPath = FieldUtils.loadPath(Constants.Auto.autoMap.get(splitCommands[i]).pathName);

        Pathfinding.setStartPosition(prevEndPoint);
        
        commandList.add
          (
            AutoBuilder.pathfindThenFollowPath(nextPath, constraints)
            .alongWith
            (
              new MoveTo(RobotContainer.s_Diffector, Constants.DiffectorConstants.coralIntakePosition)
            )
          );

        prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();    

        commandList.add(new WaitCommand(1.5));

        commandList.add(new MoveTo(RobotContainer.s_Diffector, Constants.DiffectorConstants.coralTransferPosition));
        commandList.add(new WaitUntilCommand(() -> RobotContainer.s_Diffector.atPosition(Constants.DiffectorConstants.coralTransferPosition)));
        commandList.add(new MoveTo(RobotContainer.s_Diffector, Constants.DiffectorConstants.coralStowPosition));
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

      SmartDashboard.putNumber("end point X", prevEndPoint.getX());
      SmartDashboard.putNumber("end point Y", prevEndPoint.getY());
    }

    return new SequentialCommandGroup(commandList.toArray(Command[]::new));
  }
}
