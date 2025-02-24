// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.util.ArmPathPlanner;
import frc.robot.subsystems.Diffector;

/**
 * This exists in place of the "Pathfinding" file from the Pathplanner library.
 * 
 * A custom version of "LocalADStar" has been created, with an overload constructor that allows for a
 * custom navGrid file path to be specified.
 * 
 * We have then mapped out the valid (z,φ) space of the Diffector arm, presented as a (x,y) navGrid.
 * 
 * By using this, we are (hopefully) able to use the pathfinding algorithms from Pathplanner to
 * produce an optimal path for the Diffector arm to follow.
 */
public class ArmPathPlanner 
{

  private final static double maxAbsAngle = Constants.DiffectorConstants.maxAbsAngle;
  private final static double minElevation = Constants.DiffectorConstants.minZ; 

  private static double mapScale = 0.1;
  private static double nodeSize = 0.2;

  private static CustomADStar armPathFinder;

 /**
  * Converts from arm-relative coordinates to the coordinates used by the AD* system
  * @param armElevationRotation metres over ground | total degrees anticlockwise
  * @return pathfinderXY -> arm state-space mapping
  */
 public static Translation2d fromArmRelative(Translation2d armElevationRotation)
 {
    return new Translation2d
    (
      (Math.max((armElevationRotation.getX() - minElevation), 0) / mapScale) + (2 * nodeSize),
      (armElevationRotation.getY() + maxAbsAngle) * mapScale
    );
 }

 /**
  * Converts from arm-relative coordinates to the coordinates used by the AD* system
  * @param armElevationRotation metres over ground | total degrees anticlockwise
  * @param protect ensure the elevation is safe for the given rotation (default false)
  * @return pathfinderXY -> arm state-space mapping
  */
  public static Translation2d fromArmRelative(Translation2d armElevationRotation, boolean protect)
  {
    if (protect)
    {
      return new Translation2d
      (
        (Math.max((Diffector.arm.checkPosition(armElevationRotation) - minElevation), 0) / mapScale) + (2 * nodeSize),
        (armElevationRotation.getY() + maxAbsAngle) * mapScale
      );
    }
    else
    {
      return new Translation2d
      (
        (Math.max((armElevationRotation.getX() - minElevation), 0) / mapScale) + (2 * nodeSize),
        (armElevationRotation.getY() + maxAbsAngle) * mapScale
      );
    }
  }

 /**
  * Converts from arm-relative coordinates to the coordinates used by the AD* system 
  * @param armElevation metres over ground
  * @param armRotation total degrees anticlockwise
  * @param protected ensure the elevation is safe for the given rotation (default false)
  * @return pathfinderXY -> arm state-space mapping
  */
  public static Translation2d fromArmRelative(double armElevation, double armRotation)
    {{return fromArmRelative(new Translation2d(armElevation, armRotation));}}

  /**
  * Converts from arm-relative coordinates to the coordinates used by the AD* system 
  * @param armElevation metres over ground
  * @param armRotation total degrees anticlockwise
  * @param protect ensure the elevation is safe for the given rotation (default false)
  * @return pathfinderXY -> arm state-space mapping
  */
  public static Translation2d fromArmRelative(double armElevation, double armRotation, boolean protect)
  {
    if (protect)
      {return fromArmRelative(new Translation2d(Diffector.arm.checkPosition(armElevation, armRotation), armRotation));}
    else
      {return fromArmRelative(new Translation2d(armElevation, armRotation));}
  }

 /**
  * Converts from the coordinates used by the AD* system to arm-relative coordinates
  * @param pathfinderXY arm state-space mapping
  * @return armElevationRotation -> metres over ground | total degrees anticlockwise
  */
 public static Translation2d toArmRelative(Translation2d pathfinderXY)
 {
    return new Translation2d
    (
      ((pathfinderXY.getX() - (2 * nodeSize)) * mapScale) + minElevation,
      (pathfinderXY.getY() / mapScale) - maxAbsAngle
    );
 }

  /** Ensure that a pathfinding implementation has been chosen. If not, set it to the default. */
  public static void ensureInitialized() 
  {
    if (armPathFinder == null) 
    {
      // Hasn't been initialized yet, use the default implementation
      armPathFinder = new CustomADStar("armplanner/diffector_navgrid.json");
    }
  }

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  public static boolean isNewPathAvailable() 
    {return armPathFinder.isNewPathAvailable();}

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  public static PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) 
    {return armPathFinder.getCurrentPath(constraints, goalEndState);}

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  public static void setStartPosition(Translation2d startPosition) 
    {armPathFinder.setStartPosition(startPosition);}

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  public static void setGoalPosition(Translation2d goalPosition) 
    {armPathFinder.setGoalPosition(goalPosition);}

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path if the robot is now within an obstacle.
   */
  public static void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) 
    {armPathFinder.setDynamicObstacles(obs, currentRobotPos);}
}
