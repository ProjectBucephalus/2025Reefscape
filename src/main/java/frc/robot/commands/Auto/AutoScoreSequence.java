// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreSequence extends SequentialCommandGroup 
{
  private boolean coral;
  private boolean algae;

  public AutoScoreSequence(AlgaeManipulator s_AlgaeManipulator, CoralManipulator s_CoralManipulator) 
  {
    coral = s_CoralManipulator.getStatus() == CoralManipulatorStatus.HOLDING;
    algae = s_AlgaeManipulator.getStatus() == AlgaeManipulatorStatus.HOLDING;
    
    if (coral && !algae) 
    {
      addCommands();
    }
  }
}
