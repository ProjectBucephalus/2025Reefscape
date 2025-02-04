package frc.robot.commands;

import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;
import frc.robot.util.GeoFenceObject;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command 
{    
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest
    .FieldCentric()
    .withDeadband(Constants.Control.maxThrottle * Constants.Swerve.maxSpeed * Constants.Control.stickDeadband)
    .withRotationalDeadband(Constants.Swerve.maxAngularVelocity * Constants.Control.stickDeadband)
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final SwerveRequest.RobotCentric driveRequestRoboCentric = new SwerveRequest
    .RobotCentric()
    .withDeadband(Constants.Control.maxThrottle * Constants.Swerve.maxSpeed * Constants.Control.stickDeadband)
    .withRotationalDeadband(Constants.Swerve.maxAngularVelocity * Constants.Control.stickDeadband)
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private CommandSwerveDrivetrain s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier brakeSup;
    private BooleanSupplier fieldCentricSup;
    private BooleanSupplier fencedSup;
    private Translation2d motionXY;
    private double robotRadius;
    private double robotSpeed;
    private double rotationVal;
    private double translationVal;
    private double strafeVal;
    private double brakeVal;
    private GeoFenceObject[] fieldGeoFence;
    private boolean redAlliance;

  public TeleopSwerve(CommandSwerveDrivetrain s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier brakeSup, BooleanSupplier fieldCentricSup, BooleanSupplier fencedSup) 
  {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.brakeSup = brakeSup;
    this.fieldCentricSup = fieldCentricSup;
    this.fencedSup = fencedSup;
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

  @Override
  public void execute() 
  {
    /* Get Values and apply Deadband*/
    rotationVal = rotationSup.getAsDouble();
    translationVal = translationSup.getAsDouble();
    strafeVal = strafeSup.getAsDouble();
    brakeVal = brakeSup.getAsDouble();
    motionXY = new Translation2d(translationVal, strafeVal);

    motionXY = motionXY.times(Constants.Control.maxThrottle - ((Constants.Control.maxThrottle - Constants.Control.minThrottle) * brakeVal));
    rotationVal *= (Constants.Control.maxRotThrottle - ((Constants.Control.maxRotThrottle - Constants.Control.minRotThrottle) * brakeVal));
      
    if (fieldCentricSup.getAsBoolean())
    {
      if (fencedSup.getAsBoolean())
      {
        SmartDashboard.putString("Drive State", "Fenced");
        SmartDashboard.putString("XY in:", motionXY.toString());

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
        
        SmartDashboard.putString("XY out:", motionXY.toString());
        s_Swerve.setControl
        (
          driveRequest
          .withVelocityX(motionXY.getX() * Constants.Swerve.maxSpeed)
          .withVelocityY(motionXY.getY() * Constants.Swerve.maxSpeed)
          .withRotationalRate(rotationVal * Constants.Swerve.maxAngularVelocity)
        );
      }
      else
      {   
        SmartDashboard.putString("Drive State", "Non-Fenced");
        s_Swerve.setControl
        (
          driveRequest
          .withVelocityX(motionXY.getX() * Constants.Swerve.maxSpeed)
          .withVelocityY(motionXY.getY() * Constants.Swerve.maxSpeed)
          .withRotationalRate(rotationVal * Constants.Swerve.maxAngularVelocity)
        );
      }
    }
    else
    {
      SmartDashboard.putString("Drive State", "Robot-Relative");
      s_Swerve.setControl
      (
        driveRequestRoboCentric
        .withVelocityX(motionXY.getX() * Constants.Swerve.maxSpeed)
        .withVelocityY(motionXY.getY() * Constants.Swerve.maxSpeed)
        .withRotationalRate(rotationVal * Constants.Swerve.maxAngularVelocity)
      );
    }

    motionXY = motionXY.times(Constants.Control.maxThrottle - ((Constants.Control.maxThrottle - Constants.Control.minThrottle) * brakeVal));
    rotationVal *= (Constants.Control.maxRotThrottle - ((Constants.Control.maxRotThrottle - Constants.Control.minRotThrottle) * brakeVal));
    
    if (fieldCentricSup.getAsBoolean())
    {
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
        for (int i = fieldGeoFence.length - 1; i >= 0; i--) // ERROR: Stick input seems to have been inverted for the new swerve library, verify and impliment a better fix
        {
          Translation2d inputDamping = fieldGeoFence[i].dampMotion(s_Swerve.getState().Pose.getTranslation(), motionXY.unaryMinus(), robotRadius);
          motionXY = inputDamping.unaryMinus();
        }
          s_Swerve.setControl
        (
          driveRequest
          .withVelocityX(motionXY.getX() * Constants.Swerve.maxSpeed)
          .withVelocityY(motionXY.getY() * Constants.Swerve.maxSpeed)
          .withRotationalRate(rotationVal * Constants.Swerve.maxAngularVelocity)
        );
      }
      else
      {   
        SmartDashboard.putString("Drive State", "Non-Fenced");
        s_Swerve.setControl
        (
          driveRequest
          .withVelocityX(motionXY.getX() * Constants.Swerve.maxSpeed)
          .withVelocityY(motionXY.getY() * Constants.Swerve.maxSpeed)
          .withRotationalRate(rotationVal * Constants.Swerve.maxAngularVelocity)
        );
      }
    }
    else
    {
      SmartDashboard.putString("Drive State", "Robot-Rel");
      s_Swerve.setControl
      (
        driveRequestRoboCentric
        .withVelocityX(motionXY.getX() * Constants.Swerve.maxSpeed)
        .withVelocityY(motionXY.getY() * Constants.Swerve.maxSpeed)
        .withRotationalRate(rotationVal * Constants.Swerve.maxAngularVelocity)
      );
    }
  }
}