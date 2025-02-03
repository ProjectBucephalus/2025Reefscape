// Copyright (c) FIRST and other WPILib 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;
import frc.robot.util.GeoFenceObject;

public class TargetHeadingStation extends Command 
{
  private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(Constants.Control.maxThrottle * Constants.Swerve.maxSpeed * Constants.Control.stickDeadband)
    .withRotationalDeadband(Constants.Swerve.maxAngularVelocity * Constants.Control.stickDeadband)
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private CommandSwerveDrivetrain s_Swerve;    
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier brakeSup;
  private BooleanSupplier fencedSup;
  private Translation2d motionXY;
  private DoubleSupplier ySup;
  private final GeoFenceObject[] fieldGeoFence = FieldUtils.GeoFencing.fieldGeoFence;
  private double robotRadius;
  private double robotSpeed;

  private double translationSpeed;
  private double strafeSpeed;
  private double brakeVal;
  
  private double robotY;
  private Rotation2d targetHeading;

  public TargetHeadingStation(CommandSwerveDrivetrain s_Swerve, DoubleSupplier ySup, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier brakeSup, BooleanSupplier fencedSup) 
  {
    SmartDashboard.putBoolean("Station Snap Updating", true);

    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.brakeSup = brakeSup;
    this.fencedSup = fencedSup;
    this.ySup = ySup;

    driveRequest.HeadingController.setPID(Constants.Swerve.rotationKP, Constants.Swerve.rotationKI, Constants.Swerve.rotationKD);
  }

  @Override 
  public void initialize()
    {updateTargetHeading();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    translationSpeed = translationSup.getAsDouble() * Constants.Swerve.maxSpeed;
    strafeSpeed = strafeSup.getAsDouble() * Constants.Swerve.maxSpeed;
    brakeVal = brakeSup.getAsDouble();
    motionXY = new Translation2d(translationSpeed, strafeSpeed);

    if (SmartDashboard.getBoolean("Station Snap Updating", true))
      {updateTargetHeading();}

    motionXY = motionXY.times(Constants.Control.maxThrottle - ((Constants.Control.maxThrottle - Constants.Control.minThrottle) * brakeVal));
    
    if (fencedSup.getAsBoolean())
    {
      robotSpeed = Math.hypot(s_Swerve.getState().Speeds.vxMetersPerSecond, s_Swerve.getState().Speeds.vyMetersPerSecond);
      SmartDashboard.putString("Drive State", "Fenced");
      if (robotSpeed >= FieldUtils.GeoFencing.robotSpeedThreshold)
        {robotRadius = FieldUtils.GeoFencing.robotRadiusCircumscribed;}
      else
        {robotRadius = FieldUtils.GeoFencing.robotRadiusInscribed;}
      // Read down the list of geofence objects
      // Outer wall is index 0, so has highest authority by being processed last
      for (int i = fieldGeoFence.length - 1; i >= 0; i--)
      {
        Translation2d inputDamping = fieldGeoFence[i].dampMotion(s_Swerve.getState().Pose.getTranslation(), motionXY, robotRadius);
        motionXY = inputDamping;
      }
      s_Swerve.setControl
      (
        driveRequest
        .withVelocityX(motionXY.getX())
        .withVelocityY(motionXY.getY())
        .withTargetDirection(targetHeading)
      );
    }
    else
    {   
      SmartDashboard.putString("Drive State", "Non-Fenced");
      s_Swerve.setControl
      (
        driveRequest
        .withVelocityX(motionXY.getX())
        .withVelocityY(motionXY.getY())
        .withTargetDirection(targetHeading)
      );
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return false;}

  private void updateTargetHeading()
  {
    robotY = ySup.getAsDouble();

    if (FieldUtils.isRedAlliance()) 
    {
      if (robotY >= 4.026) 
        {targetHeading = new Rotation2d(Units.degreesToRadians(-126));} 
      else 
        {targetHeading = new Rotation2d(Units.degreesToRadians(126));}
    }
    else
    {
      if (robotY >= 4.026) 
        {targetHeading = new Rotation2d(Units.degreesToRadians(126));} 
      else 
        {targetHeading = new Rotation2d(Units.degreesToRadians(-126));}
    }
  }
}
