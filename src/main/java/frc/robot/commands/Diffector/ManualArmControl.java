// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Diffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualArmControl extends Command 
{
  private final Diffector s_Diffector;
  private final DoubleSupplier deltaAngle;

  public ManualArmControl(Diffector s_Diffector, DoubleSupplier deltaAngle) 
  {
    this.s_Diffector = s_Diffector;
    this.deltaAngle = deltaAngle;
  }

  @Override
  public void execute() 
    {s_Diffector.goToAngle(s_Diffector.getAngle() + deltaAngle.getAsDouble());}

  @Override
  public boolean isFinished()
    {return false;}
}
