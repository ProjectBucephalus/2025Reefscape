// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.constants.Constants.Diffector.Presets;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Diffector;
import frc.robot.subsystems.Intake.IntakeStatus;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TransferGamePiece extends SequentialCommandGroup 
{
  Command diffectorPosCommand;
  Command intakeCommand;

  public TransferGamePiece(Diffector s_Diffector, Intake s_Intake, boolean isCoral) 
  {
    if (isCoral)
    {
      diffectorPosCommand = new MoveTo(s_Diffector, Presets.coralTransferElevation, Presets.coralTransferAngle);
      intakeCommand = new SetIntakeStatus(s_Intake, IntakeStatus.TRANSFER_CORAL);
    } 
    else
    {
      diffectorPosCommand = new MoveTo(s_Diffector, Presets.algaeTransferElevation, Presets.algaeTransferAngle);
      intakeCommand = new SetIntakeStatus(s_Intake, IntakeStatus.TRANSFER_ALGAE);
    }

    addCommands(diffectorPosCommand, intakeCommand);
  }
}
