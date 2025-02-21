// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Diffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualElevatorControl extends Command 
{
  private final Diffector s_Diffector;
  private final DoubleSupplier deltaHeight;

  public ManualElevatorControl(Diffector s_Diffector, DoubleSupplier deltaHeight) 
  {
    this.s_Diffector = s_Diffector;
    this.deltaHeight = deltaHeight;
  }

  @Override
  public void execute() 
    {s_Diffector.setElevatorTarget(s_Diffector.getElevation() + deltaHeight.getAsDouble());}

  @Override
  public boolean isFinished()
    {return false;}
}
