package frc.robot.util.controlTransmutation;
import edu.wpi.first.math.geometry.Translation2d;

/** Standard interface for input transformation functions */
public interface InputTransmuter
{
  public Translation2d process(Translation2d controlInput);
}
