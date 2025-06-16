// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class Deadband extends InputTransmuter
{
  protected double deadband;

  public Deadband()
    {this(Constants.Control.stickDeadband);}
  
  public Deadband(double deadband)
    {this.deadband = deadband;}
  
  public Translation2d process(Translation2d controlInput)
  {
    return (controlInput.getNorm() <= deadband ? Translation2d.kZero : controlInput);
  }
}