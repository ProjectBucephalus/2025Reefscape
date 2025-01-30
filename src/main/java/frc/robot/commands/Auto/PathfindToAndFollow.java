// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

public class PathfindToAndFollow extends Command 
{
  private PathPlannerPath path;
  private final PathConstraints constraints = Constants.Auto.defaultConstraints;
  private Command pathfindingCommand;
  
  public PathfindToAndFollow(String pathName, CommandSwerveDrivetrain s_Swerve) 
  {
    addRequirements(s_Swerve);

    path = FieldUtils.loadPath(pathName);
  }

  @Override
  public void initialize()
  {
    pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    pathfindingCommand.until(RobotContainer.driver.povCenter()).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return pathfindingCommand.isFinished();
  }
}
