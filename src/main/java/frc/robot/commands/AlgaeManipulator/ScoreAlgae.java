// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;
import frc.robot.subsystems.Diffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAlgae extends SequentialCommandGroup 
{
  Command diffectorPosCommand;
  Command algaeManipulatorCommand;
  Diffector s_Diffector;
  AlgaeManipulator s_AlgaeManipulator;

  public ScoreAlgae(boolean toNet, Diffector s_Diffector, AlgaeManipulator s_AlgaeManipulator) 
  {
    this.s_Diffector = s_Diffector;
    this.s_AlgaeManipulator = s_AlgaeManipulator;

    if (toNet)
    {
      diffectorPosCommand = new MoveTo(s_Diffector, Constants.Diffector.netElevation, Constants.Diffector.netAngle);
      algaeManipulatorCommand = new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.NET);
    } else 
    {
      diffectorPosCommand = new MoveTo(s_Diffector, Constants.Diffector.processorElevation, Constants.Diffector.processorAngle);
      algaeManipulatorCommand = new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.PROCESSOR);
    }
    addCommands(diffectorPosCommand, algaeManipulatorCommand);
  }
}
