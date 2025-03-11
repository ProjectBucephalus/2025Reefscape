  // Copyright (c) FIRST and other WPILib contributors./ Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.DpadOptions;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Auto.AutoMapping;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Diffector;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;

public class AutoUtils 
{
  private static final PathConstraints defaultConstraints = Constants.Auto.defaultConstraints;
  private static final PathConstraints slowedConstraints = Constants.Auto.slowedConstraints;

  /**
   * Splits a string of auto command phrases and gets the path command and robot command associated with each command phrase
   * @param commandInput The string of commands to split, seperated by commas with no spaces (e.g. "a1,rA1,p,cR3")
   * @return An array of commands, from the input command phrase string, in the same order
   */
  public static Command getCommandList(String commandInput, Diffector s_Diffector, CoralManipulator s_Coral, AlgaeManipulator s_Algae)
  {
    // Removes all space characters from the single-String command phrases, ensures it's all lowercase, and then splits it into individual strings, which are stored in an array
    String[] splitCommands = commandInput.replaceAll("//s", "").toLowerCase().split(",");
    // The arraylist that all the commands will be placed into
    ArrayList<Command> commandList = new ArrayList<>();

    // Holders for the values during each command processing
    AutoMapping autoMapValue;
    PathPlannerPath nextPath;

    // Tracks the end point of the previous path, used so each path properly pathfinds from the end point of the previous one
    Translation2d prevEndPoint = RobotContainer.swerveState.Pose.getTranslation();

    // For each command phrase, adds the associated path and then the associated command to the command list
    for (String splitCommand : splitCommands) 
    {
      switch (splitCommand.charAt(0)) 
      {
        case 'w':
          commandList.add(Commands.waitSeconds(Double.parseDouble(splitCommand.substring(1))));
          break;

        case 't':
          double targetMatchTimeElapsed = Double.parseDouble(splitCommand.substring(1));

          commandList.add
          (
            Commands.idle()
            .until
            (
              () -> Timer.getMatchTime() < (15 - targetMatchTimeElapsed)
            )
          );
          break;

        case 'r':
          nextPath = FieldUtils.loadPath(Constants.Auto.autoMap.get(splitCommand.substring(0, 2)).pathName);

          Pathfinding.setStartPosition(prevEndPoint);
          prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();

          commandList.add
          (
            Commands.parallel
            (
              AutoBuilder.pathfindThenFollowPath(nextPath, defaultConstraints),
              s_Diffector.coralScorePosCommand(prevEndPoint, Integer.parseInt(splitCommand.substring(2)))
            )
          );

          commandList.add(s_Coral.setStatusCommand(CoralManipulatorStatus.DELIVERY_SMART));
          
          if (splitCommand.charAt(2) == '4') 
          {
            commandList.add(Commands.waitSeconds(0.1));
            commandList.add(s_Diffector.coralScorePosCommand(prevEndPoint, 3).withTimeout(0.05));
          }
          
          commandList.add(Commands.waitUntil(() -> !RobotContainer.coral));
          commandList.add(Commands.waitSeconds(0.1));
          commandList.add(s_Coral.setStatusCommand(CoralManipulatorStatus.DEFAULT));
          break;

        case 'c':
          nextPath = FieldUtils.loadPath(Constants.Auto.autoMap.get(splitCommand).pathName);

          Pathfinding.setStartPosition(prevEndPoint);
          
          commandList.add
          (
            Commands.parallel
            (
              AutoBuilder.pathfindThenFollowPath(nextPath, defaultConstraints),
              s_Diffector.moveToCommand(Constants.DiffectorConstants.coralIntakePosition)
            )
          );

          prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();    

          commandList.add(Commands.waitSeconds(0.1));

          commandList.add(s_Diffector.moveToCommand(Constants.DiffectorConstants.coralTransferPosition));
          commandList.add(Commands.waitUntil(() -> s_Diffector.atPosition()));
          commandList.add(s_Diffector.moveToCommand(Constants.DiffectorConstants.coralStowPosition));
          break;

        case 'a':
          autoMapValue = Constants.Auto.autoMap.get(splitCommand);
          nextPath = FieldUtils.loadPath(autoMapValue.pathName);

          Pathfinding.setStartPosition(prevEndPoint);
          
          commandList.add
          (
            Commands.parallel
            (
              AutoBuilder.pathfindToPose(nextPath.getStartingHolonomicPose().get(), defaultConstraints),
              autoMapValue.command.get()
            )
          );
          commandList.add(AutoBuilder.followPath(nextPath));

          prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();   
          break;
      
        default:
          autoMapValue = Constants.Auto.autoMap.get(splitCommand);

          if (autoMapValue.pathName != null) 
          {       
            nextPath = FieldUtils.loadPath(autoMapValue.pathName);
    
            Pathfinding.setStartPosition(prevEndPoint);
            
            commandList.add(AutoBuilder.pathfindThenFollowPath(nextPath, defaultConstraints));
            prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();    
          }

          if (autoMapValue.command != null) 
          {
            commandList.add(autoMapValue.command.get());
          }
          break;
      }
    }

    return new SequentialCommandGroup(commandList.toArray(Command[]::new));
  }

  public static Command pathfindAndFollowCommand(String pathName, BooleanSupplier brakeSup)
  {
    PathPlannerPath path = FieldUtils.loadPath(pathName);
    boolean atPathStart = RobotContainer.swerveState.Pose.getTranslation().getDistance(path.getPoint(0).position) <= Constants.Auto.pathFollowTolerance;
    
    Command c_PathfindingCommand = 
    atPathStart 
    ? 
    AutoBuilder.followPath(path) 
    : 
    AutoBuilder.pathfindThenFollowPath(path, brakeSup.getAsBoolean() ? slowedConstraints : defaultConstraints);
    
    return c_PathfindingCommand.until(RobotContainer.driver.povCenter());
  }

  public static Command pathfindToBargeCommand(BooleanSupplier brakeSup)
  {
    Translation2d nearestBargePoint = FieldUtils.getNearestBargePoint(RobotContainer.swerveState.Pose.getTranslation());

    ArrayList<Translation2d> localList = FieldUtils.isRedAlliance() ? Constants.Auto.redBargePoints : Constants.Auto.blueBargePoints;

    int nearestBargePointNumber = localList.indexOf(nearestBargePoint) + 1;

    String pathName = ("b" + nearestBargePointNumber).toLowerCase();

    return pathfindAndFollowCommand(pathName, brakeSup);
  }

  public static Command pathfindToReefCommand(DpadOptions dpadValue, BooleanSupplier brakeSup)
  {
    int nearestReefFace = FieldUtils.getNearestReefFace(RobotContainer.swerveState.Pose.getTranslation());


    String pathName =
    switch (dpadValue) 
    {
      case CENTRE -> "a" + nearestReefFace;
    
      case LEFT, RIGHT -> 
        {
          boolean flippedFace = (nearestReefFace == 3 || nearestReefFace == 4 || nearestReefFace == 5);
          int unicodeValueOffset = 
          dpadValue == DpadOptions.RIGHT 
          ? 
          flippedFace ? 63 : 64
          : 
          flippedFace ? 64 : 63;
          
          yield "r" + (char)((nearestReefFace * 2) + unicodeValueOffset);
        }
    };

    pathName = pathName.toLowerCase();

    return pathfindAndFollowCommand(pathName, brakeSup);
  }

  public static Command pathfindToStationCommand(int stationPosition, BooleanSupplier brakeSup)
  {
    double robotY = RobotContainer.swerveState.Pose.getY();

    char stationSide = 
    FieldUtils.isRedAlliance() 
    ? 
    robotY >= 4.026 ? 'r' : 'l'
    : 
    robotY >= 4.026 ? 'l' : 'r';

    String pathName = "c" + stationSide + stationPosition;
    pathName = pathName.toLowerCase();

    return pathfindAndFollowCommand(pathName, brakeSup);
  }
}
