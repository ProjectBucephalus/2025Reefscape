// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindToReef extends Command 
{
  public enum DpadOptions{CENTRE, LEFT, RIGHT}

  private DpadOptions dpadValue;
  private Supplier<Translation2d> posSup;
  private Translation2d robotPos;

  private int nearestReefFace;
  private String pathName;
  
  private PathPlannerPath path;
  private final PathConstraints constraints = Constants.Auto.defaultConstraints;
  private Command pathfindingCommand;
 
  /** Creates a new PathfindToStation. */
  public PathfindToReef(DpadOptions dpadValue, Supplier<Translation2d> posSup, CommandSwerveDrivetrain s_Swerve) 
  {
    this.posSup = posSup;
    this.dpadValue = dpadValue;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() 
  {
    robotPos = posSup.get();

    nearestReefFace = FieldUtils.getNearestReefFace(robotPos);
    SmartDashboard.putNumber("nearest face", nearestReefFace);

    switch (dpadValue) 
    {
      case CENTRE:
        pathName = "a" + nearestReefFace;
        break;
    
      case LEFT: 
        if (nearestReefFace == 1 || nearestReefFace == 2 || nearestReefFace == 6) 
        {
          pathName = "r" + (char)((nearestReefFace * 2) + 63);
        }
        else if (nearestReefFace == 3 || nearestReefFace == 4 || nearestReefFace == 5) 
        {
          pathName = "r" + (char)((nearestReefFace * 2) + 64);
        }    
        break;

      case RIGHT:
        if (nearestReefFace == 1 || nearestReefFace == 2 || nearestReefFace == 6) 
        {
          pathName = "r" + (char)((nearestReefFace * 2) + 64);
        }
        else if (nearestReefFace == 3 || nearestReefFace == 4 || nearestReefFace == 5) 
        {
          pathName = "r" + (char)((nearestReefFace * 2) + 63);
        }      
        break;
    }

    pathName = pathName.toLowerCase();

    SmartDashboard.putString("pathname", pathName.toLowerCase());

    path = FieldUtils.loadPath(pathName);
    
    pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    pathfindingCommand.until(RobotContainer.driver.povCenter()).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return pathfindingCommand.isFinished();
  }

  public int getNearestReefFace()
  {
    return nearestReefFace;
  }
}
