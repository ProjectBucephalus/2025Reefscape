// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Diffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TransferGamePiece extends SequentialCommandGroup 
{
  private Command c_ArmCommand;

  public TransferGamePiece(Diffector s_Diffector, boolean isCoral) 
  {
    if (isCoral)
    {
      c_ArmCommand = new MoveTo(s_Diffector, Constants.DiffectorConstants.coralTransferPosition);
    } 
    else
    {
      c_ArmCommand = new MoveTo(s_Diffector, Constants.DiffectorConstants.algaeTransferPosition);
    }

    addCommands(c_ArmCommand);
  }
}
