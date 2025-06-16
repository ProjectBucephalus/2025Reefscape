// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Conversions;

/** Add your docs here. */
public class InputCurve extends InputTransmuter
{
  private double power;

  /** 
   * Parabolic curve on axis input, 
   * @param power optional, default 1 for linear
   */
  public InputCurve()
    {this(1);}
  
  /** 
   * Parabolic curve on axis input, 
   * @param power optional, default 1 for linear
   */
  public InputCurve(double power)
    {this.power = power;}

  public Translation2d process(Translation2d controlInput)
  {
    return Conversions.clamp
    (
      new Translation2d
      (
        Math.copySign(Math.pow(controlInput.getX(), power), controlInput.getX()), 
        Math.copySign(Math.pow(controlInput.getY(), power), controlInput.getY())
      )
    );
  }
}