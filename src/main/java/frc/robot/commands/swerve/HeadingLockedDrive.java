package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.SD;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HeadingLockedDrive extends SwerveCommandBase 
{
  protected Rotation2d rotationOffset;
  protected Rotation2d targetHeading;

  protected double rotationKP;
  protected double rotationKI;
  protected double rotationKD;

  protected final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest
    .FieldCentricFacingAngle()
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /** Creates a new ManualDrive. */
  public HeadingLockedDrive
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
    super(s_Swerve, translationSup, strafeSup, brakeSup, fencedSup);

    rotationKP = Swerve.rotationKP;
    rotationKI = Swerve.rotationKI;
    rotationKD = Swerve.rotationKD;

    driveRequest.HeadingController.setPID(rotationKP, rotationKI, rotationKD);

    this.targetHeading = targetHeading;
    this.rotationOffset = rotationOffset;
  }

  @Override
  public void execute()
  {
    processXY();

    updateTargetHeading();
    updateRotationPID();

    if (motionXY.getNorm() != 0)
      {SD.STATE_DRIVE.put("Heading Locked");}

    s_Swerve.setControl
    (
      driveRequest
      .withVelocityX(motionXY.getX() * Swerve.maxSpeed)
      .withVelocityY(motionXY.getY() * Swerve.maxSpeed)
      .withTargetDirection(targetHeading.plus(rotationOffset))
      .withHeadingPID(rotationKP, rotationKI, rotationKD)
    );
  }

  /** Processing to dynamicaly update the target heading */
  protected void updateTargetHeading() {}

  /** Processing to dynamicaly update the heading PID */
  protected void updateRotationPID()
  {
    if (RobotContainer.algae)
    {
      rotationKP = Swerve.rotationKPAlgae;
      rotationKI = Swerve.rotationKIAlgae;
      rotationKD = Swerve.rotationKDAlgae;
    }
    else
    {
      rotationKP = Swerve.rotationKP;
      rotationKI = Swerve.rotationKI;
      rotationKD = Swerve.rotationKD;
    }
  }
}
