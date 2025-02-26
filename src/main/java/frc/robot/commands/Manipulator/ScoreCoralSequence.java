// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.GoToCoralScorePos;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Diffector;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoralSequence extends SequentialCommandGroup 
{
  public ScoreCoralSequence(int level, Diffector s_Diffector, CoralManipulator s_CoralManipulator) 
    {addCommands(new GoToCoralScorePos(level, s_Diffector), new SetCoralStatus(s_CoralManipulator, CoralManipulatorStatus.DELIVERY_LEFT));}
}
