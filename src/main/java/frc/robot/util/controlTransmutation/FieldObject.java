package frc.robot.util.controlTransmutation;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public abstract class FieldObject extends InputTransmuter
{
  protected static Supplier<Translation2d> robotPosSup;
  protected static Translation2d robotPos;
  protected static DoubleSupplier robotRadiusSup;
  protected static double robotRadius;
  protected Translation2d centre;
  protected double radius;
  protected double buffer;
  protected double checkRadius;

  protected static final double minRadius = 0;
  protected static final double minBuffer = 0.1;

  public static void setRobotPosSup(Supplier<Translation2d> robotPosSupplier)
  {
    robotPosSup = robotPosSupplier;
  }

  public static void fetchRobotPos()
  {
    robotPos = robotPosSup.get();
    robotRadius = robotRadiusSup.getAsDouble();
  }

  public static void setRobotRadiusSup(DoubleSupplier robotRadiusSupplier)
  {
    robotRadiusSup = robotRadiusSupplier;
  }

  public double getDistance()
  {
    return centre.getDistance(robotPos) - (radius + robotRadius);
  }

  public Translation2d getCentre()
  {
    return centre;
  }

  protected boolean checkPosition()
  {
    return centre.getDistance(robotPos) <= checkRadius + robotRadius;
  }
}
