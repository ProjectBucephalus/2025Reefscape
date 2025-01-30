// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.Diffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAlgaeSequence extends SequentialCommandGroup 
{
  Command diffectorPosCommand;
  double elevation;
  double angle;
  
  public IntakeAlgaeSequence(boolean level2, Diffector s_Diffector, AlgaeManipulator s_AlgaeManipulator) 
  {
    if (level2)
    {
      elevation = Constants.Diffector.algae2Elevation;
      angle = Constants.Diffector.algae2Angel;
    }
    else
    {
      elevation = Constants.Diffector.algae1Elevation;
      angle = Constants.Diffector.algae1Angel;
    }

    addCommands(new MoveTo(s_Diffector, elevation, angle), new IntakeAlgae(s_AlgaeManipulator));
  }
}
