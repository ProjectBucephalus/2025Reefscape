// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStatus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetIntakeStatus extends Command 
{
  Intake s_Intake;
  IntakeStatus intakeStatus;

  public SetIntakeStatus(Intake s_Intake, IntakeStatus intakeStatus) 
  {
    this.s_Intake = s_Intake;
    this.intakeStatus = intakeStatus;

    addRequirements(s_Intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    s_Intake.setIntakeStatus(intakeStatus);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
