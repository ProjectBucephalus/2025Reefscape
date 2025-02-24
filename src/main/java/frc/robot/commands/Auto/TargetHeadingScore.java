// Copyright (c) FIRST and other WPILib 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;
import frc.robot.util.GeoFenceObject;

public class TargetHeadingScore extends Command 
{
  private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private CommandSwerveDrivetrain s_Swerve;    
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier brakeSup;
  private double rotationOffset;
  private BooleanSupplier fencedSup;
  private Supplier<Translation2d> posSup;
  private Translation2d motionXY;
  private GeoFenceObject[] fieldGeoFence;
  private boolean redAlliance;
  private double robotRadius;
  private double robotSpeed;

  private double translationVal;
  private double strafeVal;
  private double brakeVal;
  
  private int nearestReefFace;
  private Translation2d robotPos;
  private Translation2d nearestBargePoint;
  private double targetHeading;
  private double deadband = Constants.Control.stickDeadband;

  /** 
   * Rotation offset accounts for wanting the robot to face side-on
   */
  public TargetHeadingScore(CommandSwerveDrivetrain s_Swerve, double rotationOffset, Supplier<Translation2d> posSup, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier brakeSup, BooleanSupplier fencedSup) 
  {
    SmartDashboard.putBoolean("Heading Snap Updating", true);

    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.brakeSup = brakeSup;
    this.fencedSup = fencedSup;
    this.posSup = posSup;
    this.rotationOffset = rotationOffset;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    translationVal = translationSup.getAsDouble();
    strafeVal = strafeSup.getAsDouble();
    brakeVal = brakeSup.getAsDouble();
    motionXY = new Translation2d(translationVal, strafeVal);

    /* Apply deadbands */
    if (motionXY.getNorm() <= deadband) {motionXY = Translation2d.kZero;}

    if (SmartDashboard.getBoolean("Heading Snap Updating", true)) 
      {updateTargetHeading();}

    motionXY = motionXY.times(Constants.Control.maxThrottle - ((Constants.Control.maxThrottle - Constants.Control.minThrottle) * brakeVal));
    
    if (fencedSup.getAsBoolean() && !SmartDashboard.getBoolean("IgnoreFence", true))
    {
      SmartDashboard.putString("Drive State", "Fenced");

      robotSpeed = Math.hypot(RobotContainer.state.Speeds.vxMetersPerSecond, RobotContainer.state.Speeds.vyMetersPerSecond);
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
        Translation2d inputDamping = fieldGeoFence[i].dampMotion(RobotContainer.state.Pose.getTranslation(), motionXY, robotRadius);
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
          .withTargetDirection(new Rotation2d(Units.degreesToRadians(targetHeading + rotationOffset)))
        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {return false;}

  private void updateTargetHeading()
  {
    robotPos = posSup.get();

    nearestBargePoint = FieldUtils.getNearestBargePoint(robotPos);

    if (robotPos.getDistance(nearestBargePoint) <= Constants.GamePiecesManipulator.algaeRange) 
    {
      targetHeading = nearestBargePoint.minus(robotPos).getAngle().getDegrees();
    }
    else
    {
      nearestReefFace = FieldUtils.getNearestReefFace(robotPos);

      switch (nearestReefFace) 
      {
        case 1:
          targetHeading = 0;
          break;

        case 2:
          targetHeading = 60;
          break;

        case 3:
          targetHeading = 120;
          break;

        case 4:
          targetHeading = 180;
          break;

        case 5:
          targetHeading = -120;
          break;

        case 6:
          targetHeading = -60;
          break;
        default:
          break;
      }
    }    
  }
}
