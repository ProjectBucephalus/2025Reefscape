// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RunAutoCommandList extends SequentialCommandGroup 
{
  public RunAutoCommandList(ArrayList<Command> commandList) 
  {
    addCommands((Command[]) commandList.toArray());
  }
}
