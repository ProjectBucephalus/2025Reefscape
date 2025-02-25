// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Diffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveTo extends Command 
{
  Translation2d targetPosition;
  Diffector s_Diffector;

  public MoveTo(Diffector s_Diffector, Translation2d targetPosition) 
  {
    this.targetPosition = targetPosition;
    this.s_Diffector = s_Diffector;

    addRequirements(s_Diffector);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    s_Diffector.setElevationTarget(targetPosition.getX());
    s_Diffector.goToAngle(targetPosition.getY());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return true;}
}
