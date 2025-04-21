package frc.robot.commands.swerve;

import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.DiffectorGeometry;
import frc.robot.constants.Constants.Control;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;
import frc.robot.util.GeoFenceObject;
import frc.robot.util.SD;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;

public abstract class SwerveCommandBase extends Command
{
  protected final double deadband = Constants.Control.stickDeadband;

  protected CommandSwerveDrivetrain s_Swerve;

  protected DoubleSupplier translationSup;
  protected DoubleSupplier strafeSup;
  protected Supplier<Translation2d> robotXYSup = () -> RobotContainer.swerveState.Pose.getTranslation();
  protected DoubleSupplier brakeSup;
  protected BooleanSupplier fencedSup;
  
  protected double translationVal;
  protected double strafeVal;
  protected Translation2d motionXY;
  protected double motionXYCache;
  protected Translation2d robotXY;  
  protected double brakeVal;

  protected double robotSpeed;
  protected double robotRadius;
  protected GeoFenceObject[] fieldGeoFence;
  protected boolean redAlliance;

  /** Creates a new SwerveCommandBase. This has no rotation or drive-request methods or objects */
  public SwerveCommandBase(CommandSwerveDrivetrain s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier brakeSup, BooleanSupplier fencedSup) 
  {
    this.s_Swerve = s_Swerve;
    
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;    
    this.brakeSup = brakeSup;
    this.fencedSup = fencedSup;

    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() 
  {
    redAlliance = FieldUtils.isRedAlliance();

    if (redAlliance)
      {fieldGeoFence = FieldUtils.GeoFencing.fieldRedGeoFence;}
    else
      {fieldGeoFence = FieldUtils.GeoFencing.fieldBlueGeoFence;}

    initDriveConstraints();
  }

  /** Override to add additional initialization functionality */
  protected void initDriveConstraints() {}

  /** 
   * Processes the stick inputs using the brake and geofence
   * @return translation and strafe stick values as a Translation2d
   */
  protected Translation2d processXY()
  {
    /* Get values */
    robotXY = robotXYSup.get();
    translationVal = translationSup.getAsDouble();
    strafeVal = strafeSup.getAsDouble();
    motionXY = new Translation2d(translationVal, strafeVal);

    brakeVal = combinedBrake();

    /* Apply deadband */
    applyTranslationDeadband();

    /* Apply braking */
    motionXY = motionXY.times(MathUtil.interpolate(Control.maxThrottle, Control.minThrottle, brakeVal));
    
    if (SD.IO_LL.get())
    {
      robotSpeed = Math.hypot(RobotContainer.swerveState.Speeds.vxMetersPerSecond, RobotContainer.swerveState.Speeds.vyMetersPerSecond);
      
      updateRobotRadius();
      
      motionXYCache = motionXY.getNorm();

      // Invert processing input when on red alliance
      if (redAlliance)
        {motionXY = motionXY.unaryMinus();}

      if (RobotContainer.s_Diffector.getElevation() > DiffectorGeometry.bargeSafetyHeight && SD.IO_BARGE_PROTECTION.get())
      {
        motionXY = FieldUtils.GeoFencing.netProtectionZone.dampMotion(RobotContainer.swerveState.Pose.getTranslation(), motionXY, robotRadius);
      }
      
      if (fencedSup.getAsBoolean() && SD.IO_GEOFENCE.get())
      {   
        // Read down the list of geofence objects
        // Outer wall is index 0, so has highest authority by being processed last
        for (int i = fieldGeoFence.length - 1; i >= 0; i--)
        {
          Translation2d inputDamping = fieldGeoFence[i].dampMotion(RobotContainer.swerveState.Pose.getTranslation(), motionXY, robotRadius);
          motionXY = inputDamping;
        }

        if (SD.IO_OUTER_GEOFENCE.get())
        {
          Translation2d inputDamping = FieldUtils.GeoFencing.field.dampMotion(RobotContainer.swerveState.Pose.getTranslation(), motionXY, robotRadius);
          motionXY = inputDamping;
        }
      } 
      
      // Uninvert processing output when on red alliance
      if (redAlliance)
        {motionXY = motionXY.unaryMinus();}
      
      SD.IO_GEOFENCE_IMPACT.put(Math.max(Double.MIN_VALUE, motionXY.getNorm()) / Math.max(Double.MIN_VALUE, motionXYCache));
    }

    return motionXY;
  }

  protected void applyTranslationDeadband()
  {
    if (motionXY.getNorm() <= deadband) {motionXY = Translation2d.kZero;}
  }

  /**
   * Combines multiple brake inputs and conditions
   * @return maximum brake value, [0..1]
   */
  protected double combinedBrake()
  {
    return 
    (
      Math.max
      (
        brakeSup.getAsDouble(), 
        Math.min
        (
          (RobotContainer.s_Diffector.getElevation() - 1) * Constants.Control.armBrakeRate, 
          1
        )
      )
    );
  }

  /** Adjust the virtual radius of the robot to protect the robot under different conditions */
  protected void updateRobotRadius()
  {
    if (robotSpeed >= FieldUtils.GeoFencing.robotSpeedThreshold)
      {robotRadius = FieldUtils.GeoFencing.robotRadiusCircumscribed;}
    else
      {robotRadius = FieldUtils.GeoFencing.robotRadiusInscribed;}
  }
}
