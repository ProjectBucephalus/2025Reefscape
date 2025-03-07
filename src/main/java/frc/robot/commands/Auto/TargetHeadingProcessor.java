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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DiffectorConstants.IKGeometry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;
import frc.robot.util.GeoFenceObject;
import frc.robot.subsystems.Limelight;

public class TargetHeadingProcessor extends Command 
{
  private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private CommandSwerveDrivetrain s_Swerve;    
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier brakeSup;
  private Rotation2d rotationOffset;
  private BooleanSupplier fencedSup;
  private Translation2d motionXY;
  private DoubleSupplier xSup;
  private GeoFenceObject[] fieldGeoFence;
  private boolean redAlliance;
  private double robotRadius;
  private double robotSpeed;

  private double translationVal;
  private double strafeVal;
  private double brakeVal;
  
  private double robotX;
  private Rotation2d targetHeading;
  private double deadband = Constants.Control.stickDeadband;

  public TargetHeadingProcessor(CommandSwerveDrivetrain s_Swerve, Rotation2d rotationOffset, DoubleSupplier xSup, Rotation2d targetHeading, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier brakeSup, BooleanSupplier fencedSup) 
  {
    SmartDashboard.putBoolean("Heading Snap Updating", true);

    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.brakeSup = brakeSup;
    this.fencedSup = fencedSup;
    this.targetHeading = targetHeading;
    this.rotationOffset = rotationOffset;
    this.xSup = xSup;

    driveRequest.HeadingController.setPID(Constants.Swerve.rotationKP, Constants.Swerve.rotationKI, Constants.Swerve.rotationKD);
  }

  @Override
  public void initialize()
  {
    updateTargetHeading();
    redAlliance = FieldUtils.isRedAlliance();
    SmartDashboard.putBoolean("redAlliance", redAlliance);

    if (redAlliance)
      {fieldGeoFence = FieldUtils.GeoFencing.fieldRedGeoFence;}

    else
      {fieldGeoFence = FieldUtils.GeoFencing.fieldBlueGeoFence;}

    Limelight.setActivePOI(Limelight.TagPOI.PROCESSOR);
  }

  @Override
  public void execute() 
  {
    translationVal = translationSup.getAsDouble();
    strafeVal = strafeSup.getAsDouble();
    brakeVal = Math.max(brakeSup.getAsDouble(), Math.min((RobotContainer.s_Diffector.getElevation() - 1) * Constants.Control.armBrakeRate, 1));
    motionXY = new Translation2d(translationVal, strafeVal);

    /* Apply deadbands */
    if (motionXY.getNorm() <= deadband) {motionXY = Translation2d.kZero;}

    if (SmartDashboard.getBoolean("Heading Snap Updating", true)) 
      {updateTargetHeading();}

    motionXY = motionXY.times(Constants.Control.maxThrottle - ((Constants.Control.maxThrottle - Constants.Control.minThrottle) * brakeVal));
    
    
    robotSpeed = Math.hypot(RobotContainer.swerveState.Speeds.vxMetersPerSecond, RobotContainer.swerveState.Speeds.vyMetersPerSecond);
    if (robotSpeed >= FieldUtils.GeoFencing.robotSpeedThreshold)
    {robotRadius = FieldUtils.GeoFencing.robotRadiusCircumscribed;}
    
    else
    {robotRadius = FieldUtils.GeoFencing.robotRadiusInscribed;}
    
    // Invert processing input when on red alliance
    if (redAlliance)
    {motionXY = motionXY.unaryMinus();}

    if (RobotContainer.s_Diffector.getElevation() > IKGeometry.bargeSafetyHeight && !SmartDashboard.getBoolean("OVERIDE MODE", false)) // TODO: Copy to other drive functions as needed
    {
      motionXY = FieldUtils.GeoFencing.netProtectionZone.dampMotion(RobotContainer.swerveState.Pose.getTranslation(), motionXY, robotRadius);
    }

    if (fencedSup.getAsBoolean() && !SmartDashboard.getBoolean("IgnoreFence", false))
    {
      SmartDashboard.putString("Drive State", "Fenced");

      // Read down the list of geofence objects
      // Outer wall is index 0, so has highest authority by being processed last
      for (int i = fieldGeoFence.length - 1; i >= 0; i--) // ERROR: Stick input seems to have been inverted for the new swerve library, verify and impliment a better fix
      {
        Translation2d inputDamping = fieldGeoFence[i].dampMotion(RobotContainer.swerveState.Pose.getTranslation(), motionXY, robotRadius);
        motionXY = inputDamping;
      }
    }
    else
    {SmartDashboard.putString("Drive State", "Non-Fenced");}
    
    // Uninvert processing output when on red alliance
    if (redAlliance)
      {motionXY = motionXY.unaryMinus();}
    
    s_Swerve.setControl
      (
        driveRequest
        .withVelocityX(motionXY.getX() * Constants.Swerve.maxSpeed)
        .withVelocityY(motionXY.getY() * Constants.Swerve.maxSpeed)
        .withTargetDirection(targetHeading.plus(rotationOffset))
      );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return false;}

  private void updateTargetHeading()
  {
    robotX = xSup.getAsDouble();

    if (robotX >= 8.774) 
      {targetHeading = new Rotation2d(Units.degreesToRadians(0));} 
    else 
      {targetHeading = new Rotation2d(Units.degreesToRadians(180));}
  }
}
