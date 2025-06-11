package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.geometry.Translation2d;

/** Derived from GeoFence logic, acts as a non-directional brake within the given area */
public abstract class Buffer extends FieldObject
{
  protected double minSpeed;

  public Translation2d process(Translation2d controlInput)
  {
    return controlInput;
  }
}
