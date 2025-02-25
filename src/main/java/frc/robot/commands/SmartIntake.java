// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.DiffPIDOutput_PIDOutputModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.commands.Manipulator.SetAlgaeStatus;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.Diffector;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartIntake extends Command 
{
  private final AlgaeManipulator s_AlgaeManipulator;
  private final Diffector s_Diffector;

  /** Manages smart intake sequence */
  public SmartIntake(AlgaeManipulator s_AlgaeManipulator, Diffector s_Diffector) 
  {
    this.s_AlgaeManipulator = s_AlgaeManipulator;
    this.s_Diffector = s_Diffector;
  }

  @Override
  public void initialize()
  {
    new MoveTo(s_Diffector, Constants.DiffectorConstants.algaeIntakePosition).schedule();
  }

  @Override
  public void execute()
  {
    if (s_Diffector.atPosition())
    {
      new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.INTAKE).schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.HOLDING);
    if (RobotContainer.algae)
    {
      new MoveTo(s_Diffector, Constants.DiffectorConstants.algaeTransferPosition).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
