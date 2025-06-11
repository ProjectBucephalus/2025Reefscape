package frc.robot.util.controlTransmutation;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public abstract class FieldObject implements InputTransmuter
{
  protected static Supplier<Translation2d> robotPosSup;
  protected static Translation2d robotPos;
  protected static DoubleSupplier robotRadiusSup;
  protected static double robotRadius;
  protected Translation2d centre;
  protected double radius;
  protected double buffer;

  public static void setRobotPosSup(Supplier<Translation2d> robotPosSupplier)
  {
    robotPosSup = robotPosSupplier;
  }

  public static void fetchRobotPos()
  {
    robotPos = robotPosSup.get();
    robotRadius = robotRadiusSup.getAsDouble();
  }

  public static void setRoboteRadiusSup(DoubleSupplier robotRadiusSupplier)
  {
    robotRadiusSup = robotRadiusSupplier;
  }
}
