// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Diffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToCoralScorePos extends Command 
{
  int level;
  Diffector s_Diffector;
  Command moveCommand;

  public GoToCoralScorePos(int level, Diffector s_Diffector) 
  {
    this.s_Diffector = s_Diffector;
    this.level = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    switch (level)
    {
      case 1:
      moveCommand = new MoveTo(s_Diffector, Constants.Diffector.reef4Elevation, Constants.Diffector.reef4Angle);
      break;

      case 2:
      moveCommand = new MoveTo(s_Diffector, Constants.Diffector.reef3Elevation, Constants.Diffector.reef3Angle);
      break;

      case 3:
      moveCommand = new MoveTo(s_Diffector, Constants.Diffector.reef2Elevation, Constants.Diffector.reef2Angle);
      break;

      case 4:
      moveCommand = new MoveTo(s_Diffector, Constants.Diffector.reef1Elevation, Constants.Diffector.reef1Angle);
      break;
    }

    moveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return moveCommand.isFinished();
  }
}
