// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.AlgaeManipulator.EjectAlgae;
import frc.robot.commands.CoralManipulator.EjectCoral;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DropGamePiece extends Command 
{
  private boolean coral;
  private boolean algae;

  private AlgaeManipulator s_AlgaeManipulator;
  private CoralManipulator s_CoralManipulator;
  
  /** Creates a new DropGamePiece. */
  public DropGamePiece(AlgaeManipulator s_AlgaeManipulator, CoralManipulator s_CoralManipulator) 
  {
    this.s_AlgaeManipulator = s_AlgaeManipulator;
    this.s_CoralManipulator = s_CoralManipulator;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    coral = s_CoralManipulator.getStatus() == CoralManipulatorStatus.HOLDING;
    algae = s_AlgaeManipulator.getStatus() == AlgaeManipulatorStatus.HOLDING;

    if (coral && algae)
    {
      if (RobotContainer.copilot.rightTrigger().getAsBoolean())
      {
        new EjectAlgae(s_AlgaeManipulator).schedule();
      }
      else
      {
        new EjectCoral(s_CoralManipulator).schedule();
      }
    }
    else if (coral)
    {
      new EjectCoral(s_CoralManipulator).schedule();
    }
    else if (algae)
    {
      new EjectAlgae(s_AlgaeManipulator).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
