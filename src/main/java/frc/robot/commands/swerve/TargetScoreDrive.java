package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

public class TargetScoreDrive extends HeadingLockedDrive 
{
  private Rotation2d rotationOffset;
  private int nearestReefFace;

  private Rotation2d rB = Rotation2d.kZero;
  private Rotation2d r1 = Rotation2d.kZero;
  private Rotation2d r2 = new Rotation2d(Units.degreesToRadians(60));
  private Rotation2d r3 = new Rotation2d(Units.degreesToRadians(120));
  private Rotation2d r4 = Rotation2d.kZero;
  private Rotation2d r5 = new Rotation2d(Units.degreesToRadians(-120));
  private Rotation2d r6 = new Rotation2d(Units.degreesToRadians(-60));

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
    super(s_Swerve, translationSup, strafeSup, Rotation2d.kZero, Rotation2d.kZero, brakeSup, fencedSup);
    this.rotationOffset = rotationOffset;
  }

  @Override
  protected Rotation2d updateTargetHeading()
  {  
    if 
    (
      MathUtil.isNear(robotXY.getX(), (FieldUtils.fieldLength / 2), Constants.GamePiecesManipulator.algaeRange)
    ) 
      {targetHeading = rB.minus(rotationOffset);}
    else
    {
      nearestReefFace = FieldUtils.getNearestReefFace(robotXY);

      switch (nearestReefFace) 
      {
        case 1:
          targetHeading = r1.plus(rotationOffset);
          break;

        case 2:
          targetHeading = r2.plus(rotationOffset);
          break;

        case 3:
          targetHeading = r3.plus(rotationOffset);
          break;

        case 4:
          targetHeading = r4.minus(rotationOffset);
          break;

        case 5:
          targetHeading = r5.minus(rotationOffset);
          break;

        case 6:
          targetHeading = r6.minus(rotationOffset);
          break;
          
        default:
          break;
      }
    }

    return targetHeading;
  }
}