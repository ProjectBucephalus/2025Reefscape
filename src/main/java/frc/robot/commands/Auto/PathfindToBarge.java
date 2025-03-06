// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindToBarge extends Command 
{
  private Supplier<Translation2d> posSup;
  private Translation2d robotPos;
  
  private Translation2d nearestBargePoint;
  private int nearestBargePointNumber;
  private String pathName;
  private PathPlannerPath path;
  private final PathConstraints defaultConstraints = Constants.Auto.defaultConstraints;
  private final PathConstraints slowedConstraints = Constants.Auto.slowedConstraints;
  private ArrayList<Translation2d> localList;
  private Command pathfindingCommand;
  private BooleanSupplier brakeSup;
 
  public PathfindToBarge(Supplier<Translation2d> posSup, CommandSwerveDrivetrain s_Swerve, BooleanSupplier brakeSup) 
  {
    this.posSup = posSup;
    this.brakeSup = brakeSup;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() 
  {
    robotPos = posSup.get();

    nearestBargePoint = FieldUtils.getNearestBargePoint(robotPos);

    if (FieldUtils.isRedAlliance()) 
    {   
      localList = Constants.Auto.redBargePoints;
    }
    else
    {
      localList = Constants.Auto.blueBargePoints;
    }

    nearestBargePointNumber = localList.indexOf(nearestBargePoint) + 1;

    pathName = "b" + nearestBargePointNumber;

    pathName = pathName.toLowerCase();

    path = FieldUtils.loadPath(pathName);
    
    if (brakeSup.getAsBoolean())
      {pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, slowedConstraints);}
    else
      {pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, defaultConstraints);}
    pathfindingCommand.until(RobotContainer.driver.povCenter()).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return pathfindingCommand.isFinished();}
}
