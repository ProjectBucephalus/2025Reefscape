package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TargetCageDrive extends HeadingLockedDrive
{
  public TargetCageDrive
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
  protected void applyTranslationDeadband() 
  {
    double translationOut = Math.abs(translationVal) < Math.abs(strafeVal) ? 0 : translationVal;
    double strafeOut = Math.abs(strafeVal) < Math.abs(translationVal) ? 0 : strafeVal;

    motionXY = new Translation2d(translationOut, strafeOut);

    if (motionXY.getNorm() <= deadband) {motionXY = Translation2d.kZero;}
  }
}
