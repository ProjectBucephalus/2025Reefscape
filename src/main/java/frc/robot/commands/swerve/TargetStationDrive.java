package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TargetStationDrive extends HeadingLockedDrive 
{
  /** Creates a new TargetStationDrive. */
  public TargetStationDrive
  (
    CommandSwerveDrivetrain swerve, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup, 
    Rotation2d rotationOffset, 
    DoubleSupplier brakeSup, 
    BooleanSupplier fencedSup
  ) 
  {
    super(swerve, translationSup, strafeSup, Rotation2d.kZero, rotationOffset, brakeSup, fencedSup);
  }

  @Override
  protected void updateTargetHeading()
  {
    if (redAlliance) 
    {
      if (robotXY.getY() >= 4.026) 
        {targetHeading = new Rotation2d(Units.degreesToRadians(-126));} 

      else 
        {targetHeading = new Rotation2d(Units.degreesToRadians(126));}
    }
    else
    {
      if (robotXY.getY() >= 4.026) 
        {targetHeading = new Rotation2d(Units.degreesToRadians(126));} 
        
      else 
        {targetHeading = new Rotation2d(Units.degreesToRadians(-126));}
    }
  }
}