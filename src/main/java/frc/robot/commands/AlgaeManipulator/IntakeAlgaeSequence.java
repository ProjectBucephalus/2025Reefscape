// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Diffector.GoToAlgaeIntakePos;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;
import frc.robot.subsystems.Diffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAlgaeSequence extends SequentialCommandGroup 
{ 
  public IntakeAlgaeSequence(boolean level2, Diffector s_Diffector, AlgaeManipulator s_AlgaeManipulator) 
    {addCommands(new GoToAlgaeIntakePos(level2, s_Diffector), new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.INTAKE));}
}
