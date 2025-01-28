// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindToReef extends Command 
{
  public enum DpadOptions{CENTRE, LEFT, RIGHT}

  private DpadOptions dpadValue;
  private Translation2d robotPos;

  private int nearestReefFace;
  private String pathName;
  
  private PathPlannerPath path;
  private final PathConstraints constraints = Constants.Auto.defaultConstraints;
  private Command pathfindingCommand;
 
  /** Creates a new PathfindToStation. */
  public PathfindToReef(DpadOptions dpadInput, Translation2d robotPos) 
  {
    this.robotPos = robotPos;
    dpadValue = dpadInput;

    nearestReefFace = Constants.Auto.reefMidPoints.indexOf(this.robotPos.nearest(Constants.Auto.reefMidPoints));
    nearestReefFace = (int)MathUtil.inputModulus(nearestReefFace, 1, 6);

    switch (dpadValue) 
    {
      case CENTRE:
        pathName = "a" + nearestReefFace;
        break;
    
      case LEFT: 
        if (nearestReefFace == (1 | 2 | 6)) 
        {
          pathName = "r" + (char)((nearestReefFace * 2) + 63);
        }
        else if (nearestReefFace == (3 | 4 | 5)) 
        {
          pathName = "r" + (char)((nearestReefFace * 2) + 64);
        }    
        break;

      case RIGHT:
        if (nearestReefFace == (1 | 2 | 6)) 
        {
          pathName = "r" + (char)((nearestReefFace * 2) + 64);
        }
        else if (nearestReefFace == (3 | 4 | 5)) 
        {
          pathName = "r" + (char)((nearestReefFace * 2) + 63);
        }      
        break;
    }

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
    pathfindingCommand.andThen
    (
      new TargetHeading
      (
        RobotContainer.s_Swerve, 
        path.getGoalEndState().rotation(), 
        () -> -RobotContainer.driver.getRawAxis(RobotContainer.translationAxis), 
        () -> -RobotContainer.driver.getRawAxis(RobotContainer.strafeAxis), 
        () -> -RobotContainer.driver.getRawAxis(RobotContainer.rotationAxis), 
        () -> RobotContainer.driver.getRawAxis(RobotContainer.brakeAxis),
        () -> !RobotContainer.driver.leftStick().getAsBoolean()
      )
    );
    pathfindingCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
