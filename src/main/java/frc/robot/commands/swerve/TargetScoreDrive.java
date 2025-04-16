package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

public class TargetScoreDrive extends HeadingLockedDrive 
{
  private Rotation2d rotationOffsetBase;
  private int nearestReefFace;

  /** Creates a new TargetScoreDrive. */
  public TargetScoreDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup,
    Rotation2d rotationOffset, 
    DoubleSupplier brakeSup, 
    BooleanSupplier fencedSup
  ) 
  {
    super(s_Swerve, translationSup, strafeSup, Rotation2d.kZero, rotationOffset, brakeSup, fencedSup);
    this.rotationOffsetBase = rotationOffset;
  }

  @Override
  protected void applyTranslationDeadband() 
  {
    if (MathUtil.isNear(robotXY.getX(), (FieldUtils.fieldLength / 2), Constants.Control.driveSnappingRange)) 
    {
      double translationOut = Math.abs(translationVal) < Math.abs(strafeVal) ? 0 : translationVal;
      double strafeOut = Math.abs(strafeVal) < Math.abs(translationVal) ? 0 : strafeVal;

      motionXY = new Translation2d(translationOut, strafeOut);
    }
    
    if (motionXY.getNorm() <= deadband) {motionXY = Translation2d.kZero;}
  }

  @Override
  protected void updateTargetHeading()
  { 
    if 
    (
      FieldUtils.GeoFencing.reefBlue.getDistance(robotXY) >= FieldUtils.GeoFencing.robotRadiusCircumscribed &&
      FieldUtils.GeoFencing.reefRed.getDistance(robotXY) >= FieldUtils.GeoFencing.robotRadiusCircumscribed
    )
    {
      if (MathUtil.isNear(robotXY.getX(), (FieldUtils.fieldLength / 2), Constants.Manipulators.algaeRange)) 
      {
        targetHeading = Rotation2d.kZero;
        super.rotationOffset = this.rotationOffsetBase.unaryMinus();
      }
      else
      {
        nearestReefFace = FieldUtils.getNearestReefFace(robotXY);

        switch (nearestReefFace) 
        {
          case 1:
            targetHeading = Rotation2d.kZero;
            super.rotationOffset = this.rotationOffsetBase;
            break;

          case 2:
            targetHeading = new Rotation2d(Units.degreesToRadians(60));
            super.rotationOffset = this.rotationOffsetBase.unaryMinus();
            break;

          case 3:
            targetHeading = new Rotation2d(Units.degreesToRadians(120));
            super.rotationOffset = this.rotationOffsetBase.unaryMinus();
            break;

          case 4:
            targetHeading = Rotation2d.kZero;
            super.rotationOffset = this.rotationOffsetBase.unaryMinus();
            break;

          case 5:
            targetHeading = new Rotation2d(Units.degreesToRadians(-120));
            super.rotationOffset = this.rotationOffsetBase;
            break;

          case 6:
            targetHeading = new Rotation2d(Units.degreesToRadians(-60));
            super.rotationOffset = this.rotationOffsetBase;
            break;
            
          default:
            break;
        }
      }
    }
  }
}