// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Diffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveManual extends Command 
{
  DoubleSupplier height;
  DoubleSupplier angle;
  Diffector s_Diffector;

  public MoveManual(Diffector s_Diffector, DoubleSupplier heightSup, DoubleSupplier angleSup) 
  {
    this.height = heightSup;
    this.angle = angleSup;
    this.s_Diffector = s_Diffector;

    addRequirements(s_Diffector);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    s_Diffector.setElevatorTarget(s_Diffector.getElevatorPos() + (height.getAsDouble() * Constants.Diffector.manualElevationScalar));
    s_Diffector.goToAngle(s_Diffector.getArmPos() + (angle.getAsDouble() * Constants.Diffector.manualAngleScalar));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return false;}
}
