// Copyright (c) FIRST and other WPILib 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralStatus extends Command 
{
  CoralManipulator s_CoralManipulator;
  CoralManipulatorStatus s_CoralManipulatorStatus;

  public SetCoralStatus(CoralManipulator s_CoralManipulator, CoralManipulatorStatus s_CoralManipulatorStatus) 
  {
    this.s_CoralManipulator = s_CoralManipulator;
    this.s_CoralManipulatorStatus = s_CoralManipulatorStatus;

    addRequirements(s_CoralManipulator);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
    {s_CoralManipulator.setCoralManipulatorStatus(s_CoralManipulatorStatus);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return true;}
}
