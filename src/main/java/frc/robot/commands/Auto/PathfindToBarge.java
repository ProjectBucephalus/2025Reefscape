// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private final PathConstraints constraints = Constants.Auto.defaultConstraints;
  private Command pathfindingCommand;
 
  public PathfindToBarge(Supplier<Translation2d> posSup, CommandSwerveDrivetrain s_Swerve) 
  {
    this.posSup = posSup;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() 
  {
    robotPos = posSup.get();

    nearestBargePoint = FieldUtils.getNearestBargePoint(robotPos);
    
    pathfindingCommand = AutoBuilder.pathfindToPose(new Pose2d(nearestBargePoint.plus(new Translation2d(1.5, 0)), Rotation2d.kZero), constraints);
    pathfindingCommand.until(RobotContainer.driver.povCenter()).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return pathfindingCommand.isFinished();}
}
