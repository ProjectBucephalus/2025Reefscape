// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command 
{
  CoralManipulator s_CoralManipulator;
  private boolean isFinished;

  public IntakeCoral(CoralManipulator s_CoralManipulator) 
  {
    this.s_CoralManipulator = s_CoralManipulator;

    addRequirements(s_CoralManipulator);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (!s_CoralManipulator.getCoralBeamBreak1State() || !s_CoralManipulator.getCoralBeamBreak2State())
    {
      s_CoralManipulator.setCoralManipulatorStatus(CoralManipulatorStatus.HOLDING);
      isFinished = true;
    }
    else
    {
      s_CoralManipulator.setCoralManipulatorStatus(CoralManipulatorStatus.INTAKE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return isFinished;
  }
}
