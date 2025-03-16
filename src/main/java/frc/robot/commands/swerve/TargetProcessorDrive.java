package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  protected Rotation2d updateTargetHeading()
  {
    if (FieldUtils.isRedAlliance()) 
    {
      if (robotXY.getX() >= 8.774) 
        {targetHeading = new Rotation2d(Units.degreesToRadians(-90));} 

      else 
        {targetHeading = new Rotation2d(Units.degreesToRadians(90));}
    }
    else
    {
      if (robotXY.getX() >= 8.774) 
        {targetHeading = new Rotation2d(Units.degreesToRadians(90));} 
        
      else 
        {targetHeading = new Rotation2d(Units.degreesToRadians(-90));}
    }

    return targetHeading;
  }
}
