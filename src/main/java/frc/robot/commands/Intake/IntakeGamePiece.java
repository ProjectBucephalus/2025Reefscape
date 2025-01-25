// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;  

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStatus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeGamePiece extends Command 
{
Intake s_Intake;
boolean isCoral;
  public IntakeGamePiece(Intake s_Intake, boolean isCoral) 
  {
  this.s_Intake = s_Intake;
  this.isCoral = isCoral;

  addRequirements(s_Intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (isCoral)
    {
      s_Intake.setIntakeStatus(IntakeStatus.INTAKE_CORAL);
    } else
    {
      s_Intake.setIntakeStatus(IntakeStatus.INTAKE_ALGAE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
