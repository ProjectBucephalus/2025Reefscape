// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private Translation2d target;

  public ScoreAlgaeSequence(boolean toNet, Diffector s_Diffector, AlgaeManipulator s_AlgaeManipulator) 
  {
    target = toNet ? Constants.DiffectorConstants.netPosition : Constants.DiffectorConstants.processorPosition;
      
    addCommands(new MoveTo(s_Diffector, target), new WaitUntilCommand(() -> s_Diffector.atPosition()), new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.EJECT));
  }
}
