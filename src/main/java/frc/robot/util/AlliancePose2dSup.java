package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlliancePose2dSup implements Supplier<Pose2d>
{
  private final Pose2d poseBlue;

  /**
   * Constructs a new AlliancePose2dSup based on blue origin
   * @param x
   * @param y
   * @param rotation
   */
  public AlliancePose2dSup(double x, double y, double rotation)
    {poseBlue = Conversions.buildPose(x, y, rotation);}

  @Override
  public Pose2d get() 
  {
    return FieldUtils.isRedAlliance() ? poseBlue.rotateAround(FieldUtils.fieldCentre, Rotation2d.k180deg) : poseBlue;
  }
}
