// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Diffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualDiffectorControl extends Command 
{
  private final Diffector s_Diffector;
  private final DoubleSupplier elevationSup;
  private final DoubleSupplier rotationSup;

  private double rotation;
  private double elevation;

  public ManualDiffectorControl(Diffector s_Diffector, DoubleSupplier translationSup, DoubleSupplier rotationSup) 
  {
    this.s_Diffector = s_Diffector;
    this.elevationSup = translationSup;
    this.rotationSup = rotationSup;
  }

  @Override
  public void execute() 
  {
    rotation = rotationSup.getAsDouble();
    elevation = elevationSup.getAsDouble();
    
    if (Math.abs(elevation) > 2 * Math.abs(rotation)) 
    {
      rotation = 0;
    }
    if (Math.abs(rotation) > 2 * Math.abs(elevation))
    {
      elevation = 0;
    }

    s_Diffector.setManualDiffectorValues(elevation, rotation);
  }

  @Override
  public void end(boolean interrupted) 
  {
    s_Diffector.setManualDiffectorValues(0, 0);
  }

  @Override
  public boolean isFinished()
    {return false;}
}
