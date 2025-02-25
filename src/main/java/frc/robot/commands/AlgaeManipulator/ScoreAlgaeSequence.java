// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.*;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;
import frc.robot.subsystems.Diffector;
import frc.robot.constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAlgaeSequence extends SequentialCommandGroup 
{
  Command diffectorPosCommand;

  public ScoreAlgaeSequence(boolean toNet, Diffector s_Diffector, AlgaeManipulator s_AlgaeManipulator) 
  {
    if (toNet)
    {
      diffectorPosCommand = new MoveTo(s_Diffector, Constants.DiffectorConstants.netPosition);
    } 
    else 
    {
      diffectorPosCommand = new MoveTo(s_Diffector, Constants.DiffectorConstants.processorPosition);
    }
    addCommands(diffectorPosCommand, new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.EJECT));
  }
}
