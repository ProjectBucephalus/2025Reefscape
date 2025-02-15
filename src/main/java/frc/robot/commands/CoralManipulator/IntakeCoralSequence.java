// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Diffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCoralSequence extends SequentialCommandGroup 
{
  public IntakeCoralSequence(Diffector s_Diffector, CoralManipulator s_CoralManipulator) 
    {addCommands(new MoveTo(s_Diffector, Constants.DiffectorConstants.coralStationElevation, Constants.DiffectorConstants.coralStationAngle), new IntakeCoral(s_CoralManipulator));}
}
