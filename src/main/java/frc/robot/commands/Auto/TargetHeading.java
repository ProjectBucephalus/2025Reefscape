// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;
import frc.robot.util.GeoFenceObject;

public class TargetHeading extends Command 
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
  private GeoFenceObject[] fieldGeoFence;
  private boolean redAlliance;
  private double robotRadius;
  private double robotSpeed;

  private double translationVal;
  private double strafeVal;
  private double brakeVal;
  
  private Rotation2d targetHeading;

  public TargetHeading(CommandSwerveDrivetrain s_Swerve, Rotation2d targetHeading, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier brakeSup, BooleanSupplier fencedSup) 
  {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.brakeSup = brakeSup;
    this.fencedSup = fencedSup;
    this.targetHeading = targetHeading;

    driveRequest.HeadingController.setPID(Constants.Swerve.rotationKP, Constants.Swerve.rotationKI, Constants.Swerve.rotationKD);
  }

  @Override
  public void initialize()
  {
    redAlliance = FieldUtils.isRedAlliance();
    SmartDashboard.putBoolean("redAlliance", redAlliance);
    if (redAlliance)
      {fieldGeoFence = FieldUtils.GeoFencing.fieldRedGeoFence;}
    else
      {fieldGeoFence = FieldUtils.GeoFencing.fieldBlueGeoFence;}
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    translationVal = translationSup.getAsDouble();
    strafeVal = strafeSup.getAsDouble();
    brakeVal = brakeSup.getAsDouble();
    motionXY = new Translation2d(translationVal, strafeVal);

    motionXY = motionXY.times(Constants.Control.maxThrottle - ((Constants.Control.maxThrottle - Constants.Control.minThrottle) * brakeVal));
    
    if (fencedSup.getAsBoolean())
    {
      SmartDashboard.putString("Drive State", "Fenced");

      robotSpeed = Math.hypot(s_Swerve.getState().Speeds.vxMetersPerSecond, s_Swerve.getState().Speeds.vyMetersPerSecond);
      if (robotSpeed >= FieldUtils.GeoFencing.robotSpeedThreshold)
        {robotRadius = FieldUtils.GeoFencing.robotRadiusCircumscribed;}
      else
        {robotRadius = FieldUtils.GeoFencing.robotRadiusInscribed;}

      // Invert processing input when on red alliance
      if (redAlliance)
        {motionXY = motionXY.unaryMinus();}

      // Read down the list of geofence objects
      // Outer wall is index 0, so has highest authority by being processed last
      for (int i = fieldGeoFence.length - 1; i >= 0; i--) // ERROR: Stick input seems to have been inverted for the new swerve library, verify and impliment a better fix
      {
        Translation2d inputDamping = fieldGeoFence[i].dampMotion(s_Swerve.getState().Pose.getTranslation(), motionXY, robotRadius);
        motionXY = inputDamping;
      }

      // Uninvert processing output when on red alliance
      if (redAlliance)
        {motionXY = motionXY.unaryMinus();}
    }
    else
      {SmartDashboard.putString("Drive State", "Non-Fenced");}
    
    s_Swerve.setControl
    (
      driveRequest
      .withVelocityX(motionXY.getX() * Constants.Swerve.maxSpeed)
      .withVelocityY(motionXY.getY() * Constants.Swerve.maxSpeed)
      .withTargetDirection(targetHeading)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return false;}
}
