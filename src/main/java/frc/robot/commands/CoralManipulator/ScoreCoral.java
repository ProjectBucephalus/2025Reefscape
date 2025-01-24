// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Diffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoral extends SequentialCommandGroup 
{
  Command diffectorPosCommand;
  Diffector s_Diffector;
  CoralManipulator s_CoralManipulator;
  public ScoreCoral(int level, Diffector s_Diffector, CoralManipulator s_CoralManipulator) 
  {
    this.s_Diffector = s_Diffector;
    this.s_CoralManipulator = s_CoralManipulator;

    switch(level)
    {
      case 1:
      diffectorPosCommand = new MoveTo(s_Diffector, 0, 0);
      break;

      case 2:
      diffectorPosCommand = new MoveTo(s_Diffector, 0, 0);
      break;

      case 3:
      diffectorPosCommand = new MoveTo(s_Diffector, 0, 0);
      break;

      case 4:
      diffectorPosCommand = new MoveTo(s_Diffector, 0, 0);
      break;
    }
    addCommands(diffectorPosCommand, new OutTakeCoral(s_CoralManipulator));
  }
}
