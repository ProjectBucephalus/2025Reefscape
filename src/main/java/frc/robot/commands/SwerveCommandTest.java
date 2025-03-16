package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DiffectorConstants.IKGeometry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.FieldUtils;
import frc.robot.util.GeoFenceObject;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.MathUtil;

//public class SwerveCommandTest extends Command 
//{    
//  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest
//  // Teleop
//    .FieldCentric() 
//  // TargetHeading*
//    .FieldCentricFacingAngle() 
//    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
//    .withSteerRequestType(SteerRequestType.MotionMagicExpo);
//
//  private final SwerveRequest.RobotCentric driveRequestRoboCentric = new SwerveRequest
//    .RobotCentric()
//    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
//    .withSteerRequestType(SteerRequestType.MotionMagicExpo);
//
//    private CommandSwerveDrivetrain s_Swerve;    
//    private DoubleSupplier translationSup;
//    private DoubleSupplier strafeSup;
//    private DoubleSupplier rotationSup;
//    private DoubleSupplier brakeSup;
//    private BooleanSupplier fencedSup;
//    private Translation2d motionXY;
//    private double robotRadius;
//    private double robotSpeed;
//    private double rotationVal;
//    private double translationVal;
//    private double strafeVal;
//    private double brakeVal;
//    private GeoFenceObject[] fieldGeoFence;
//    private boolean redAlliance;
//    private double deadband = Constants.Control.stickDeadband;
//
//    //Target Heading
//    
//
//    //TargetHeadingScore
//   private Supplier<Translation2d> robotXYSup;
//   private int nearestReefFace;
//   private Translation2d robotXY;
//   private Translation2d nearestBargePoint;
//  
//  public SwerveCommandTest(CommandSwerveDrivetrain s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier brakeSup, BooleanSupplier fencedSup) 
//  {
//    this.s_Swerve = s_Swerve;
//    addRequirements(s_Swerve);
//
//    robotXYSup = () -> RobotContainer.swerveState.Pose.getTranslation();
//
//    this.translationSup = translationSup;
//    this.strafeSup = strafeSup;
//    // Teleop
//    this.rotationSup = rotationSup;
//
//    this.brakeSup = brakeSup;
//    this.fencedSup = fencedSup;
//
//    //Target Heading
//    this.targetHeading = targetHeading;
//    this.rotationOffset = rotationOffset;
//    driveRequest.HeadingController.setPID(Constants.Swerve.rotationKP, Constants.Swerve.rotationKI, Constants.Swerve.rotationKD);
//
//    //TargetHeadingScore
//    SmartDashboard.putBoolean("Heading Snap Updating", true);
//  }
//
//    @Override
//    public void initialize()
//    {
//      //TargetHeadingProccessor
//      updateTargetHeading();
//
//      redAlliance = FieldUtils.isRedAlliance();
//
//      //Target Heading 
//      SmartDashboard.putBoolean("redAlliance", redAlliance);
//     
//      if (redAlliance)
//        {fieldGeoFence = FieldUtils.GeoFencing.fieldRedGeoFence;}
//
//      else
//        {fieldGeoFence = FieldUtils.GeoFencing.fieldBlueGeoFence;}
//
//      Limelight.setActivePOI(Limelight.TagPOI.REEF);
//
//      //TargetHeadingProccessor
//      Limelight.setActivePOI(Limelight.TagPOI.PROCESSOR);
//
//      //TargetHeadingStation
//      Limelight.setActivePOI(Limelight.TagPOI.CORALSTATION);
//    }
//
//  @Override
//  public void execute() 
//  {
//    /* Get values */
//    robotXY = robotXYSup.get();
//    // Teleop
//    rotationVal = rotationSup.getAsDouble();
//
//    translationVal = translationSup.getAsDouble();
//    strafeVal = strafeSup.getAsDouble();
//    brakeVal = Math.max(brakeSup.getAsDouble(), Math.min((RobotContainer.s_Diffector.getElevation() - 1) * Constants.Control.armBrakeRate, 1));
//    motionXY = new Translation2d(translationVal, strafeVal);
//
//    /* Apply deadbands */
//    if (motionXY.getNorm() <= deadband) {motionXY = Translation2d.kZero;}
//    // Teleop
//    if (Math.abs(rotationVal) <= deadband) {rotationVal = 0;}
//
//    //TargetHeadingProccessor
//    if (SmartDashboard.getBoolean("Heading Snap Updating", true)) 
//    {updateTargetHeading();}
//
//    /* Apply braking */
//    motionXY = motionXY.times(Constants.Control.maxThrottle - ((Constants.Control.maxThrottle - Constants.Control.minThrottle) * brakeVal));
//    rotationVal *= (Constants.Control.maxRotThrottle - ((Constants.Control.maxRotThrottle - Constants.Control.minRotThrottle) * brakeVal));
//    
//    
//    robotSpeed = Math.hypot(RobotContainer.swerveState.Speeds.vxMetersPerSecond, RobotContainer.swerveState.Speeds.vyMetersPerSecond);
//    if (robotSpeed >= FieldUtils.GeoFencing.robotSpeedThreshold)
//    {robotRadius = FieldUtils.GeoFencing.robotRadiusCircumscribed;}
//    
//    else
//    {robotRadius = FieldUtils.GeoFencing.robotRadiusInscribed;}
//    
//    // Invert processing input when on red alliance
//    if (redAlliance)
//      {motionXY = motionXY.unaryMinus();}
//
//    if (RobotContainer.s_Diffector.getElevation() > IKGeometry.bargeSafetyHeight && !SmartDashboard.getBoolean("OVERIDE MODE", false)) // TODO: Copy to other drive functions as needed
//    {
//      motionXY = FieldUtils.GeoFencing.netProtectionZone.dampMotion(RobotContainer.swerveState.Pose.getTranslation(), motionXY, robotRadius);
//    }
//    
//    if (fencedSup.getAsBoolean() && !SmartDashboard.getBoolean("IgnoreFence", false))
//    {
//      SmartDashboard.putString("Drive State", "Fenced");
//      
//      // Read down the list of geofence objects
//      // Outer wall is index 0, so has highest authority by being processed last
//      for (int i = fieldGeoFence.length - 1; i >= 0; i--) // ERROR: Stick input seems to have been inverted for the new swerve library, verify and impliment a better fix
//      {
//        Translation2d inputDamping = fieldGeoFence[i].dampMotion(RobotContainer.swerveState.Pose.getTranslation(), motionXY, robotRadius);
//        motionXY = inputDamping;
//      }
//    } 
//    else 
//    {SmartDashboard.putString("Drive State", "Non-Fenced");}
//    
//    // Uninvert processing output when on red alliance
//    if (redAlliance)
//      {motionXY = motionXY.unaryMinus();}
//    
//    s_Swerve.setControl
//    (
//      driveRequest
//      .withVelocityX(motionXY.getX() * Constants.Swerve.maxSpeed)
//      .withVelocityY(motionXY.getY() * Constants.Swerve.maxSpeed)
//      .withRotationalRate(rotationVal * Constants.Swerve.maxAngularVelocity)
//    );
//  }
//    /* Target Heading \/
//    s_Swerve.setControl
//    (
//      driveRequest
//      .withVelocityX(motionXY.getX() * Constants.Swerve.maxSpeed)
//      .withVelocityY(motionXY.getY() * Constants.Swerve.maxSpeed)
//      .withTargetDirection(targetHeading.plus(rotationOffset))
//    );*/
//
//    //TargetHeadingProccessor
//    private void updateTargetHeading()
//    {  
//      if (FieldUtils.isRedAlliance()) 
//      {
//        if (robotXY.getX() >= 8.774) 
//          {targetHeading = new Rotation2d(Units.degreesToRadians(-90));} 
//  
//        else 
//          {targetHeading = new Rotation2d(Units.degreesToRadians(90));}
//      }
//      else
//      {
//        if (robotXY.getX() >= 8.774) 
//          {targetHeading = new Rotation2d(Units.degreesToRadians(90));} 
//          
//        else 
//          {targetHeading = new Rotation2d(Units.degreesToRadians(-90));}
//      }
//    }
//
//    // TargetHeadingScore
//    private void updateTargetHeading()
//    {
//      nearestBargePoint = FieldUtils.getNearestBargePoint(robotXY);
//  
//      // TODO: Confirm this works for both aliances
//      if 
//      (
//        MathUtil.isNear(robotXY.getX(), (FieldUtils.fieldLength / 2), Constants.GamePiecesManipulator.algaeRange)
//      ) 
//        {targetHeading = 0 - rotationOffset;}
//      else
//      {
//        nearestReefFace = FieldUtils.getNearestReefFace(robotXY);
//  
//        switch (nearestReefFace) 
//        {
//          case 1:
//            targetHeading = 0 + rotationOffset;
//            break;
//  
//          case 2:
//            targetHeading = 60 + rotationOffset;
//            break;
//  
//          case 3:
//            targetHeading = 120 + rotationOffset;
//            break;
//  
//          case 4:
//            targetHeading = 0 - rotationOffset;
//            break;
//  
//          case 5:
//            targetHeading = -120 - rotationOffset;
//            break;
//  
//          case 6:
//            targetHeading = -60 - rotationOffset;
//            break;
//            
//          default:
//            break;
//        }
//      }  
//    }
//
//    // TargetHeadingStation
//    private void updateTargetHeading()
//    {
//      if (FieldUtils.isRedAlliance()) 
//      {
//        if (robotXY.getY() >= 4.026) 
//          {targetHeading = new Rotation2d(Units.degreesToRadians(-126));} 
//  
//        else 
//          {targetHeading = new Rotation2d(Units.degreesToRadians(126));}
//      }
//      else
//      {
//        if (robotXY.getY() >= 4.026) 
//          {targetHeading = new Rotation2d(Units.degreesToRadians(126));} 
//          
//        else 
//          {targetHeading = new Rotation2d(Units.degreesToRadians(-126));}
//      }
//    }
//  }
//}
//