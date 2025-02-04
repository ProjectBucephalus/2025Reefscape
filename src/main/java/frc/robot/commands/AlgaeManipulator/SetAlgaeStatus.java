// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlgaeStatus extends Command 
{
  AlgaeManipulator s_AlgaeManipulator;
  AlgaeManipulatorStatus algaeManipulatorStatus;

  public SetAlgaeStatus(AlgaeManipulator s_AlgaeManipulator, AlgaeManipulatorStatus algaeManipulatorStatus) 
  {
    this.s_AlgaeManipulator = s_AlgaeManipulator;
    this.algaeManipulatorStatus = algaeManipulatorStatus;

    addRequirements(s_AlgaeManipulator);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
    {s_AlgaeManipulator.setAlgaeManipulatorStatus(algaeManipulatorStatus);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
