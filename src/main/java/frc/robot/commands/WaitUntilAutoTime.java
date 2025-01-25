// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitUntilAutoTime extends Command {
  private double targetTime;

  /** Waits until a certain match time has elapsed */
  public WaitUntilAutoTime(double timeElapsed) 
  {
    targetTime = 15 - timeElapsed;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return Timer.getMatchTime() < targetTime;
  }
}
