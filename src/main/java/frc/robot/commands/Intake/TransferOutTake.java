// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStatus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TransferOutTake extends Command 
{
  Intake s_Intake;
  IntakeStatus s_IntakeStatus;
  public TransferOutTake(Intake s_Intake, IntakeStatus s_IntakeStatus) 
  {
    this.s_Intake = s_Intake;
    this.s_IntakeStatus = s_IntakeStatus;

    addRequirements(s_Intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    s_Intake.setIntakeStatus(s_IntakeStatus);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
