// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.MoveTo;
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
  Diffector s_Diffector;
  Intake s_Intake;
  IntakeStatus s_IntakeStatus;
  boolean isCoral;
  public TransferGamePiece(Diffector s_Diffector, Intake s_Intake, IntakeStatus s_IntakeStatus, boolean isCoral) 
  {
    this.isCoral = isCoral;
    this.s_Intake = s_Intake;
    this.s_IntakeStatus = s_IntakeStatus;
    this.s_Diffector = s_Diffector;
    if (isCoral)
    {
      diffectorPosCommand = new MoveTo(s_Diffector, 0.1, 0.1);
      intakeCommand = new TransferOutTake(s_Intake, s_IntakeStatus);
    } else
    {
      diffectorPosCommand = new MoveTo(s_Diffector, 0.1, 0.1);
      intakeCommand = new TransferOutTake(s_Intake, s_IntakeStatus);
    }

    addCommands(diffectorPosCommand, intakeCommand);
  }
}
