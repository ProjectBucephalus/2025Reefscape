// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Diffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToAlgaeIntakePos extends Command 
{
  double elevation;
  double angle;
  boolean level2;
  Diffector s_Diffector;
  Command moveCommand;

  public GoToAlgaeIntakePos(boolean level2, Diffector s_Diffector) 
  {
    this.s_Diffector = s_Diffector;
    this.level2 = level2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if (level2)
    {
      elevation = Constants.DiffectorConstants.algae2Elevation;
      angle = Constants.DiffectorConstants.algae2Angle;
    }
    else
    {
      elevation = Constants.DiffectorConstants.algae1Elevation;
      angle = Constants.DiffectorConstants.algae1Angle;
    }

    moveCommand = new MoveTo(s_Diffector, elevation, angle);
    moveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
    {return moveCommand.isFinished();}
}
