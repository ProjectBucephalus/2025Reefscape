package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

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
  protected void updateTargetHeading() 
  {
    rotationOffset = 
    MathUtil.isNear(robotXY.getX(), (FieldUtils.fieldLength / 2), Constants.Control.cageFaceDistance) ? 
      Rotation2d.kZero :
      Rotation2d.kCW_90deg;
  }

  @Override
  protected void applyTranslationDeadband() 
  {
    if (MathUtil.isNear(robotXY.getX(), (FieldUtils.fieldLength / 2), Constants.Manipulators.algaeRange)) 
    {
      double translationOut = Math.abs(translationVal) < Math.abs(strafeVal) ? 0 : translationVal;
      double strafeOut = Math.abs(strafeVal) < Math.abs(translationVal) ? 0 : strafeVal;

      motionXY = new Translation2d(translationOut, strafeOut);
    }
    
    if (motionXY.getNorm() <= deadband) {motionXY = Translation2d.kZero;}
  }

  @Override
  protected void updateRobotRadius()
  {
    if (MathUtil.isNear(robotXY.getX(), (FieldUtils.fieldLength / 2), Constants.Control.driveSnappingRange))
      {robotRadius = FieldUtils.GeoFencing.robotRadiusMinimum;}
    else if (robotSpeed >= FieldUtils.GeoFencing.robotSpeedThreshold)
      {robotRadius = FieldUtils.GeoFencing.robotRadiusCircumscribed;}
    else
      {robotRadius = FieldUtils.GeoFencing.robotRadiusInscribed;}
  }
}
