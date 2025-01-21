package frc.robot.commands;

import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.GeoFenceObject;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {    
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Control.maxThrottle * Constants.Swerve.maxSpeed * Constants.Control.stickDeadband)
        .withRotationalDeadband(Constants.Swerve.maxAngularVelocity * Constants.Control.stickDeadband)
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.RobotCentric driveRequestRoboCentric = new SwerveRequest.RobotCentric()
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
    private final GeoFenceObject[] fieldGeoFence = FieldConstants.GeoFencing.fieldGeoFence;
    private double robotRadius;
    private double robotSpeed;

    private double rotationVal;
    private double translationVal;
    private double strafeVal;
    private double brakeVal;

    public TeleopSwerve(CommandSwerveDrivetrain s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier brakeSup, BooleanSupplier fieldCentricSup, BooleanSupplier fencedSup) {
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
                robotSpeed = Math.hypot(s_Swerve.getState().Speeds.vxMetersPerSecond, s_Swerve.getState().Speeds.vyMetersPerSecond);
                SmartDashboard.putString("Drive State", "Fenced");
                if (robotSpeed >= FieldConstants.GeoFencing.robotSpeedThreshold)
                {
                    robotRadius = FieldConstants.GeoFencing.robotRadiusCircumscribed;
                }
                else
                {
                    robotRadius = FieldConstants.GeoFencing.robotRadiusInscribed;
                }
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