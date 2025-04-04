  // Copyright (c) FIRST and other WPILib contributors./ Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.DpadOptions;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Constants.Auto.AutoMapping;
import frc.robot.constants.Constants.DiffectorConstants.Presets;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Diffector;
import frc.robot.subsystems.CoralManipulator.Status;

public class AutoUtils 
{
  private static final PathConstraints defaultConstraints = Constants.Auto.defaultConstraints;
  private static final PathConstraints slowedConstraints = Constants.Auto.slowedConstraints;

  private static Translation2d prevEndPoint;

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
    prevEndPoint = RobotContainer.swerveState.Pose.getTranslation();

    // For each command phrase, adds the associated path and then the associated command to the command list
    for (String splitCommand : splitCommands) 
    {
      switch (splitCommand.charAt(0)) 
      {
        case 'g':
          int seperatorIndex = splitCommand.indexOf(":");
          Translation2d posTarget = 
          new Translation2d
          (
            MathUtil.clamp(Double.parseDouble(splitCommand.substring(1, seperatorIndex)), 0.5, (FieldUtils.fieldLength / 2) - 0.5), 
            MathUtil.clamp(Double.parseDouble(splitCommand.substring(seperatorIndex + 1)), 0.5, FieldUtils.fieldWidth - 0.5)
          );

          if (FieldUtils.isRedAlliance()) 
          {
            posTarget = posTarget.rotateAround(new Translation2d(FieldUtils.fieldLength / 2, FieldUtils.fieldWidth / 2), Rotation2d.k180deg);
          }

          commandList.add(AutoBuilder.pathfindToPose(new Pose2d(posTarget, FieldUtils.isRedAlliance() ? Rotation2d.kZero: Rotation2d.k180deg), defaultConstraints));
          prevEndPoint = posTarget;
          break;

        case 'w':
          commandList.add(Commands.waitSeconds(Double.parseDouble(splitCommand.substring(1))));
          break;

        case 't':
          double targetMatchTimeElapsed = Double.parseDouble(splitCommand.substring(1));

          commandList.add(Commands.waitUntil(() -> Timer.getMatchTime() < (15 - targetMatchTimeElapsed)));
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
              s_Diffector.coralScorePosCommandUndeferredAllianceLocked(() -> prevEndPoint, Integer.parseInt(splitCommand.substring(2)))
            )
          );

          commandList.add(s_Coral.setStatusCommand(CoralManipulator.Status.DELIVERY_SMART));
          
          if (splitCommand.charAt(2) == '4') 
          {
            commandList.add
            (
              Commands.sequence
              (
                Commands.waitSeconds(0.2),
                s_Diffector.runOnce(() -> s_Diffector.goToAngle(0))
              )
            );
          }
          
          commandList.add(Commands.waitUntil(() -> !RobotContainer.coral));
          commandList.add(s_Coral.setStatusCommand(CoralManipulator.Status.DEFAULT));
          break;

        case 'c':
          nextPath = FieldUtils.loadPath(Constants.Auto.autoMap.get(splitCommand).pathName);
          ArmPos armPos = splitCommand.charAt(1) == 'r' ? Presets.coralIntakePosition.port() : Presets.coralIntakePosition.stbd();

          Pathfinding.setStartPosition(prevEndPoint);

          commandList.add
          (
            Commands.parallel
            (
              AutoBuilder.pathfindThenFollowPath(nextPath, defaultConstraints),
              s_Diffector.moveAndWaitCommand(armPos)
            )
          );

          prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();    

          commandList.add(s_Coral.setStatusCommand(Status.INTAKE));
          commandList.add(Commands.waitUntil(() -> RobotContainer.coral));
          commandList.add(s_Diffector.moveToCommand(Presets.coralStowPosition));
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

  public static Command pathfindAndFollowCommand(Supplier<String> pathNameSup, BooleanSupplier brakeSup)
  {
    PathPlannerPath path = FieldUtils.loadPath(pathNameSup.get());
    BooleanSupplier atPathStart = () -> RobotContainer.swerveState.Pose.getTranslation().getDistance(path.getPoint(0).position) <= Constants.Auto.atPosTolerance;
    
    return 
    Commands.either
    (
      AutoBuilder.followPath(path), 
      Commands.either
      (
        AutoBuilder.pathfindThenFollowPath(path, slowedConstraints), 
        AutoBuilder.pathfindThenFollowPath(path, defaultConstraints), 
        brakeSup
      ),
      atPathStart
    )
    .until(RobotContainer.driver.povCenter())
    .withName("PathfindAndFollow");
  }

  public static Supplier<String> getBargePathName()
  {
    return 
    () ->
    {
      Translation2d nearestBargePoint = FieldUtils.getNearestBargePoint(RobotContainer.swerveState.Pose.getTranslation());

      ArrayList<Translation2d> localList = FieldUtils.isRedAlliance() ? FieldConstants.redBargePoints : FieldConstants.blueBargePoints;
  
      int nearestBargePointNumber = localList.indexOf(nearestBargePoint) + 1;
  
      return ("b" + nearestBargePointNumber).toLowerCase();
    };
  }

  public static Supplier<String> getReefPathName(DpadOptions dpadValue)
  {
    return 
    () ->
    {
      int nearestReefFace = FieldUtils.getNearestReefFace(RobotContainer.swerveState.Pose.getTranslation());

      String pathName =
      switch (dpadValue) 
      {
        case CENTRE -> "a" + nearestReefFace;
      
        case LEFT, RIGHT -> 
          {
            boolean flippedFace = (nearestReefFace == 4);
            int unicodeValueOffset = 
            dpadValue == DpadOptions.RIGHT 
            ? 
            flippedFace ? 63 : 64
            : 
            flippedFace ? 64 : 63;
            
            yield "r" + (char)((nearestReefFace * 2) + unicodeValueOffset);
          }
      };
      return pathName.toLowerCase();
    };
  }

  public static Supplier<String> getStationPathName(int stationPosition)
  {
    return 
    () ->
    {
      double robotY = RobotContainer.swerveState.Pose.getY();

      char stationSide = 
      FieldUtils.isRedAlliance() 
      ? 
      robotY >= 4.026 ? 'r' : 'l'
      : 
      robotY >= 4.026 ? 'l' : 'r';

      return ("c" + stationSide + stationPosition).toLowerCase();
    };
  }

  public static Supplier<String> getClimbPathName()
  {
    return
    () ->
    {
      double distanceFromFieldCenter = Math.abs(RobotContainer.swerveState.Pose.getY() - (FieldUtils.fieldWidth / 2));
      int nearestCageNumber;
      if (distanceFromFieldCenter > 2.724) 
        {nearestCageNumber = 3;}
      else if (distanceFromFieldCenter > 1.5985)
        {nearestCageNumber = 2;}
      else 
        {nearestCageNumber = 1;}

      return ("cage" + nearestCageNumber + "climb").toLowerCase();
    };
  }

  public static Command autoScoreSequenceCommand(Diffector s_Diffector, AlgaeManipulator s_Algae, CoralManipulator s_Coral, IntSupplier reefLevel, BooleanSupplier brakeSup, IntSupplier povAngle, BooleanSupplier cancelTrigger)
  {
    int nearestReefFace = FieldUtils.getNearestReefFace(RobotContainer.swerveState.Pose.getTranslation());
    PathPlannerPath algaePath = FieldUtils.loadPath("a" + nearestReefFace);
    int coralLevel =
    switch (reefLevel.getAsInt())
    {
      case 1, 2, 3, 4 -> reefLevel.getAsInt();
      default -> nearestReefFace % 2 == 0 ? 2 : 3;
    };
    DpadOptions dpadValue = 
    switch (povAngle.getAsInt())
    {
      case 90 -> DpadOptions.RIGHT;
      case 270 -> DpadOptions.LEFT;
      default -> DpadOptions.LEFT;
    };

    return
    Commands.sequence
    (
      Commands.parallel
      (
        AutoBuilder.pathfindToPose(algaePath.getStartingHolonomicPose().get(), brakeSup.getAsBoolean() ? slowedConstraints : defaultConstraints),
        intakeAlgaeSequenceCommand(s_Diffector, s_Algae, nearestReefFace)
      ),
      AutoBuilder.followPath(algaePath),
      s_Diffector.coralScorePosCommand(coralLevel),
      pathfindAndFollowCommand(getReefPathName(dpadValue), brakeSup),
      s_Coral.setStatusCommand(CoralManipulator.Status.DELIVERY_SMART)
    )
    .until(cancelTrigger);
  }
  
  public static Command intakeAlgaeSequenceCommand(Diffector s_Diffector, AlgaeManipulator s_Algae, int nearestReefFace)
  {
    return 
    Commands.sequence
    (
      s_Diffector.algaeIntakePosCommand(nearestReefFace),
      s_Algae.setStatusCommand(AlgaeManipulator.Status.INTAKE)
    );
  }

  public static Command scoreAlgaeSequenceCommand(Diffector s_Diffector, AlgaeManipulator s_Algae, boolean net)
  {
    return
    Commands.sequence
    (
      s_Diffector.moveAndWaitCommand(net ? Presets.netPosition : Presets.processorPosition.port()), 
      s_Algae.setStatusCommand(AlgaeManipulator.Status.EJECT)
    );
  }

  public static Command ejectAlgaeSequenceCommand(Diffector s_Diffector, AlgaeManipulator s_Algae, Supplier<Translation2d> posSup)
  {
    int nearestReefFace = FieldUtils.getNearestReefFace(posSup.get());
    boolean portReefFace = (nearestReefFace == 5 || nearestReefFace == 6);

    ArmPos target = 
    nearestReefFace % 2 == 0 
    ?
    portReefFace ? Presets.algae2Position.stbd() : Presets.algae2Position.port()
    :
    portReefFace ? Presets.algae3Position.stbd() : Presets.algae3Position.port();

    return
    Commands.sequence
    (
      s_Diffector.moveAndWaitCommand(target), 
      s_Algae.setStatusCommand(AlgaeManipulator.Status.EJECT)
    );
  }
}
