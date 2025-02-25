// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
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
  private BooleanSupplier  algaeModifier;
  
  /** Creates a new DropGamePiece. */
  public DropGamePiece(AlgaeManipulator s_AlgaeManipulator, CoralManipulator s_CoralManipulator, BooleanSupplier algaeModifier) 
  {
    this.s_AlgaeManipulator = s_AlgaeManipulator;
    this.s_CoralManipulator = s_CoralManipulator;
    this.algaeModifier = algaeModifier;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    coral = s_CoralManipulator.getStatus() == CoralManipulatorStatus.DEFAULT;
    algae = s_AlgaeManipulator.getStatus() == AlgaeManipulatorStatus.HOLDING;

    if (coral && algae)
    {
      if (algaeModifier.getAsBoolean())
        {new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.EJECT).schedule();}
      else
        {new SetCoralStatus(s_CoralManipulator, CoralManipulatorStatus.DELIVERY_LEFT).schedule();}
    }
    else if (coral)
      {new SetCoralStatus(s_CoralManipulator, CoralManipulatorStatus.DELIVERY_LEFT).schedule();}
    else if (algae)
      {new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.EJECT).schedule();}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return true;}
}
