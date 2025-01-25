// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OutTakeCoral extends Command 
{
  CoralManipulator s_CoralManipulator;
  public OutTakeCoral(CoralManipulator s_CoralManipulator) 
  {
    this.s_CoralManipulator = s_CoralManipulator;

    addRequirements(s_CoralManipulator);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    s_CoralManipulator.setCoralManipulatorSpeed(0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
