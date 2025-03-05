// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Diffector;
import frc.robot.util.FieldUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToCoralScorePos extends Command 
{
  private int level;
  private Translation2d target;
  private Diffector s_Diffector;
  private Command c_MoveCommand;
  private int nearestReefFace;
  private Translation2d robotPos;
  private Supplier<Translation2d> posSup;

  public GoToCoralScorePos(int level, Diffector s_Diffector, Supplier<Translation2d> posSup) 
  {
    this.s_Diffector = s_Diffector;
    this.level = level;
    this.posSup = posSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    robotPos = posSup.get();
    nearestReefFace = FieldUtils.getNearestReefFace(robotPos);

    switch (level)
    {
      case 4:
        if (nearestReefFace == 5 || nearestReefFace == 6) 
        {
          target = Constants.DiffectorConstants.coral4PortPosition;
        }
        else
        {
          target = Constants.DiffectorConstants.coral4StbdPosition;
        }
        break;

      case 3:
        if (nearestReefFace == 5 || nearestReefFace == 6) 
        {
          target = Constants.DiffectorConstants.coral3PortPosition;
        }
        else
        {
          target = Constants.DiffectorConstants.coral3StbdPosition;
        }
        break;

      case 2:
        if (nearestReefFace == 5 || nearestReefFace == 6) 
        {
          target = Constants.DiffectorConstants.coral2PortPosition;
        }
        else
        {
          target = Constants.DiffectorConstants.coral2StbdPosition;
        }
        break;

      case 1:
        if (nearestReefFace == 5 || nearestReefFace == 6) 
        {
          target = Constants.DiffectorConstants.coral1PortPosition;
        }
        else
        {
          target = Constants.DiffectorConstants.coral1StbdPosition;
        }
        break;
    }

    c_MoveCommand = new MoveTo(s_Diffector, target);
    c_MoveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
    {return c_MoveCommand.isFinished();}
}
