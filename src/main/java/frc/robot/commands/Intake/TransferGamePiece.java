// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Diffector;
import frc.robot.subsystems.Intake.IntakeStatus;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TransferGamePiece extends SequentialCommandGroup 
{
  private ArrayList<Command> commandSet = new ArrayList<Command>();

  public TransferGamePiece(Diffector s_Diffector, Intake s_Intake, boolean isCoral) 
  {
    if (isCoral)
    {
      commandSet.add(new MoveTo(s_Diffector, Constants.DiffectorConstants.coralTransferPosition));
    } 
    else
    {
      commandSet.add(new MoveTo(s_Diffector, Constants.DiffectorConstants.algaeTransferPosition));
      commandSet.add(new SetIntakeStatus(s_Intake, IntakeStatus.TRANSFER_ALGAE));
    }

    addCommands(commandSet.toArray(Command[]::new));
  }
}
