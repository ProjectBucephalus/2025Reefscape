package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

public class TargetProcessorDrive extends HeadingLockedDrive 
{
  /** Creates a new TargetProcessorDrive. */
  public TargetProcessorDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup, 
    Rotation2d targetHeading, 
    Rotation2d rotationOffset, 
    DoubleSupplier brakeSup, 
    BooleanSupplier fencedSup
  ) 
  {
    super(s_Swerve, translationSup, strafeSup, targetHeading, rotationOffset, brakeSup, fencedSup);
  }

  @Override
  protected void updateTargetHeading()
  {
    targetHeading = FieldUtils.isRedAlliance() ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg;
  }
}
