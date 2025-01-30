// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStatus;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndStow extends SequentialCommandGroup 
{
  Command intakeCommand;
  public IntakeAndStow(int intakeStatus, Intake s_Intake) {
    switch (intakeStatus) 
    {
      case 1:
        intakeCommand = new SetIntakeStatus(s_Intake, IntakeStatus.INTAKE_ALGAE);
        break;
    
      case 2:
        intakeCommand = new SetIntakeStatus(s_Intake, IntakeStatus.INTAKE_CORAL);
        break;

      case 3:
        intakeCommand = new SetIntakeStatus(s_Intake, IntakeStatus.EJECT_ALGAE);
        break;

      case 4:
        intakeCommand = new SetIntakeStatus(s_Intake, IntakeStatus.EJECT_CORAL);
        break;
            
      case 5:
        intakeCommand = new SetIntakeStatus(s_Intake, IntakeStatus.STOWED);
        break;

      case 6:
        intakeCommand = new SetIntakeStatus(s_Intake, IntakeStatus.STAND_BY);
        break;
    }
    addCommands(intakeCommand);
  }
}
