// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class Brake extends InputTransmuter
{
  private DoubleSupplier brakeAxis;
  private double max;
  private double min;

  public Brake()
  {
    brakeAxis = null;
    max = Constants.Control.maxThrottle;
    min = Constants.Control.minThrottle;
  }

  public Brake(DoubleSupplier brakeAxis, double maxThrottle, double minThrottle)
  {
    this.brakeAxis = brakeAxis;
    max = maxThrottle;
    min = minThrottle;
  }

  public double get()
  {
    return brakeAxis == null ? max : MathUtil.interpolate(max, min, brakeAxis.getAsDouble());
  }

  public Translation2d process(Translation2d controlInput)
  {
    return controlInput.times(get());
  }

  public Brake withBrakeAxis(DoubleSupplier brakeAxis)
  {
    this.brakeAxis = brakeAxis;
    return this;
  }
}