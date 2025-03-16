package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TargetStationDrive extends HeadingLockedDrive 
{
  private Rotation2d rStationL = new Rotation2d(Units.degreesToRadians(126));
  private Rotation2d rStationR = new Rotation2d(Units.degreesToRadians(-126));

  /** Creates a new TargetStationDrive. */
  public TargetStationDrive
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
  }

  @Override
  protected Rotation2d updateTargetHeading()
  {
    if (redAlliance) 
    {
      if (robotXY.getY() >= 4.026) 
        {targetHeading = rStationR;} 

      else 
        {targetHeading = rStationL;}
    }
    else
    {
      if (robotXY.getY() >= 4.026) 
        {targetHeading = rStationL;} 
        
      else 
        {targetHeading = rStationR;}
    }

    return targetHeading;
  }
}