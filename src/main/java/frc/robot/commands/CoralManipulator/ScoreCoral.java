// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Diffector;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoral extends SequentialCommandGroup 
{
  Command diffectorPosCommand;

  public ScoreCoral(int level, Diffector s_Diffector, CoralManipulator s_CoralManipulator) 
  {
    switch(level)
    {
      case 1:
        diffectorPosCommand = new MoveTo(s_Diffector, Constants.DiffectorConstants.reef4Elevation, Constants.DiffectorConstants.reef4Angle);
        break;

      case 2:
        diffectorPosCommand = new MoveTo(s_Diffector, Constants.DiffectorConstants.reef3Elevation, Constants.DiffectorConstants.reef3Angle);
        break;

      case 3:
        diffectorPosCommand = new MoveTo(s_Diffector, Constants.DiffectorConstants.reef2Elevation, Constants.DiffectorConstants.reef2Angle);
        break;

      case 4:
        diffectorPosCommand = new MoveTo(s_Diffector, Constants.DiffectorConstants.reef1Elevation, Constants.DiffectorConstants.reef1Angle);
        break;
    }
    addCommands(diffectorPosCommand, new SetCoralStatus(s_CoralManipulator, CoralManipulatorStatus.DELIVERY));
  }
}
