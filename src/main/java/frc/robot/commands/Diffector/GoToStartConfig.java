// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Diffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToStartConfig extends SequentialCommandGroup {
  public GoToStartConfig(Diffector s_Diffector) 
  {
    if (s_Diffector.getElevatorPos() >= 0.5)
    {
      addCommands(Commands.runOnce(() -> s_Diffector.setElevatorTarget(0.6), s_Diffector));
    }  
    
    addCommands( 
      Commands.runOnce(() -> s_Diffector.unwind(), s_Diffector).unless(() -> s_Diffector.unwind()), 
      Commands.runOnce(() -> s_Diffector.setElevatorTarget(0), s_Diffector));
  }
}
