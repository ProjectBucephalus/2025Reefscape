// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindToStation extends Command 
{
  private final PathConstraints constraints = Constants.Auto.defaultConstraints;
  
  private DoubleSupplier ySup;
  private int stationPosition;

  private double robotY;

  private char stationSide;
  private String pathName;

  private PathPlannerPath path;
  private Command pathfindingCommand;
 
  /** Creates a new PathfindToStation. */
  public PathfindToStation(int stationPosition, DoubleSupplier ySup, CommandSwerveDrivetrain s_Swerve) 
  {
    this.ySup = ySup;
    this.stationPosition = stationPosition;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize()
  {
    robotY = ySup.getAsDouble();

    if (FieldUtils.isRedAlliance()) 
    {
      if (robotY >= 4.026) 
        {stationSide = 'r';}
      else 
        {stationSide = 'l';}
    }
    else
    {
      if (robotY >= 4.026) 
        {stationSide = 'l';}
      else 
        {stationSide = 'r';}
    }

    pathName = "c" + stationSide + stationPosition;

    path = FieldUtils.loadPath(pathName);

    pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    pathfindingCommand.until(RobotContainer.driver.povCenter()).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return pathfindingCommand.isFinished();}
}
