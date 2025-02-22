package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.AlgaeManipulator.IntakeAlgaeSequence;
import frc.robot.commands.AlgaeManipulator.ScoreAlgaeSequence;
import frc.robot.commands.CoralManipulator.IntakeCoralSequence;
import frc.robot.util.FieldUtils;

public final class Constants 
{
  public static final class RumblerConstants 
  {
    public static final double driverDefault = 1;
    public static final double copilotDefault = 1;  
  }

  public static final class Control
  {
    public static final double stickDeadband = 0.15;
    /** Normal maximum robot speed, relative to maximum uncapped speed */
    public static final double maxThrottle = 1;
    /** Minimum robot speed when braking, relative to maximum uncapped speed */
    public static final double minThrottle = 0.1;
    /** Normal maximum rotational robot speed, relative to maximum uncapped rotational speed */
    public static final double maxRotThrottle = 1;
    /** Minimum rotational robot speed when braking, relative to maximum uncapped rotational speed */
    public static final double minRotThrottle = 0.5;
  }

  public static final class Vision
  {
    public static final int[] validIDs = 
    {
      //1, 2, 3,               // Red Human Player Stations
      //4, 5,                  // Red Barge
      //6, 7, 8, 9, 10, 11,    // Red Reef
      12, 13, 16,            // Blue Human Player Stations
      14, 15,                // Blue Barge
      17, 18, 19, 20, 21, 22 // Blue Reef
    };
  }

  public static final class Swerve
  {
    /** Centre-centre distance (length and width) between wheels, metres */
    public static final double drivebaseWidth = 0.616;
    public static final double initialHeading = 0;

    /* Drive PID Values */
    public static final double driveKP = 5.4; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Rotation Control PID Values */
    public static final double rotationKP = 3; //TODO: Tune to robot
    public static final double rotationKI = 0;
    public static final double rotationKD = 0;

    /* Swerve Limit Values */
    /** Meters per Second */
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Radians per Second */
    public static final double maxAngularVelocity = 4;
  }

  public static final class Auto
  {   
  /** m/s */
    public static final double pathplannerMaxSpeed = 2;
    /** m/s^2 */
    public static final double pathplannerMaxAcceleration = 2;
    /** degrees/s */
    public static final double pathplannerMaxAngularSpeed = 720;
    /** degrees/s^2 */
    public static final double pathplannerMaxAngularAcceleration = 1050;
    public static final PathConstraints defaultConstraints = new PathConstraints
      (pathplannerMaxSpeed, pathplannerMaxAcceleration, pathplannerMaxAngularSpeed, pathplannerMaxAngularAcceleration);
    
    public static final Map<Translation2d, Integer> reefMidPointMap = new HashMap<>(6)
    {
      {
        put(new Translation2d(3.658, 4.026), 1);
        put(new Translation2d(4.073, 3.306), 2);
        put(new Translation2d(4.905, 3.306), 3);
        put(new Translation2d(5.321, 4.026), 4);
        put(new Translation2d(4.905, 4.746), 5);
        put(new Translation2d(4.073, 4.746), 6);
      }
    };

    public static class AutoMapping
    {
      public final String pathName;
      public final Supplier<Command> command;

      public AutoMapping(String pathName, Supplier<Command> command)
      {
        this.pathName = pathName;
        this.command = command;
      }
    }

    /* Maps all dynamic auto paths to the name used for them in the dashbord. Dashboard Name, Path Name */
    public static final Map<String, AutoMapping> autoMap = new HashMap<>(34)
    {
      {
        put("ra" , new AutoMapping("ra" , null));
        put("rb" , new AutoMapping("rb" , null));
        put("rc" , new AutoMapping("rc" , null));
        put("rd" , new AutoMapping("rd" , null));
        put("re" , new AutoMapping("re" , null));
        put("rf" , new AutoMapping("rf" , null));
        put("rg" , new AutoMapping("rg" , null));
        put("rh" , new AutoMapping("rh" , null));
        put("ri" , new AutoMapping("ri" , null));
        put("rj" , new AutoMapping("rj" , null));
        put("rk" , new AutoMapping("rk" , null));
        put("rl" , new AutoMapping("rl" , null));
        put("cl1", new AutoMapping("cl1", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cl2", new AutoMapping("cl2", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cl3", new AutoMapping("cl3", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cl4", new AutoMapping("cl4", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cl5", new AutoMapping("cl5", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cl6", new AutoMapping("cl6", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cl7", new AutoMapping("cl7", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cl8", new AutoMapping("cl8", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cl9", new AutoMapping("cl9", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr1", new AutoMapping("cr1", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr2", new AutoMapping("cr2", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr3", new AutoMapping("cr3", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr4", new AutoMapping("cr4", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr5", new AutoMapping("cr5", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr6", new AutoMapping("cr6", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr7", new AutoMapping("cr7", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr8", new AutoMapping("cr8", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr9", new AutoMapping("cr9", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("a1" , new AutoMapping("a1" , () -> new IntakeAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));
        put("a2" , new AutoMapping("a2" , () -> new IntakeAlgaeSequence(false, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));                
        put("a3" , new AutoMapping("a3" , () -> new IntakeAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));                
        put("a4" , new AutoMapping("a4" , () -> new IntakeAlgaeSequence(false, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));                
        put("a5" , new AutoMapping("a5" , () -> new IntakeAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));                
        put("a6" , new AutoMapping("a6" , () -> new IntakeAlgaeSequence(false, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));
        put("b1" , new AutoMapping("b1" , () -> new ScoreAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));
        put("b2" , new AutoMapping("b2" , () -> new ScoreAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));
        put("b3" , new AutoMapping("b3" , () -> new ScoreAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));
        put("p"  , new AutoMapping("p"  , () -> new ScoreAlgaeSequence(false, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));                
      }
    };

    public static final ArrayList<Translation2d> reefBlueMidPoints = FieldUtils.GeoFencing.reefBlue.getMidPoints();
    public static final ArrayList<Translation2d> reefRedMidPoints = FieldUtils.GeoFencing.reefRed.getMidPoints();

    public static final String defaultAuto = "t5,cR5,w3.5,cR5";
  }

  public static final class DiffectorConstants
  {
    public static final double diffectorMotorKGEmpty = 0.225;
    public static final double diffectorMotorKSEmpty = 0.05;
    public static final double diffectorMotorKVEmpty = 0.58;
    public static final double diffectorMotorKPEmpty = 100;
    public static final double diffectorMotorKIEmpty = 0;
    public static final double diffectorMotorKDEmpty = 0;

    public static final double diffectorMotorKGOneItem = 0;
    public static final double diffectorMotorKSOneItem = 0;
    public static final double diffectorMotorKVOneItem = 0;
    public static final double diffectorMotorKPOneItem = 3;
    public static final double diffectorMotorKIOneItem = 0;
    public static final double diffectorMotorKDOneItem = 0;

    public static final double diffectorMotorKGTwoItem = 0;
    public static final double diffectorMotorKSTwoItem = 0;
    public static final double diffectorMotorKVTwoItem = 0;
    public static final double diffectorMotorKPTwoItem = 3;
    public static final double diffectorMotorKITwoItem = 0;
    public static final double diffectorMotorKDTwoItem = 0;

    /** Desired cruise speed OF MOTOR, RPS */
    public static final double diffectorMotionMagicCruise = 100; //100
    /** Desired acceleration OF MOTOR, RPS^2 */
    public static final double diffectorMotionMagicAccel  = 600; //600

    public static final double coralElevatorLowTheshold = 0;
    public static final double coralElevatorHighThreshold = 0;
    public static final double algaeElevatorLowTheshold = 0;
    public static final double algaeElevatorHighThreshold = 0;
    public static final double climberElevatorLowTheshold = 0;
    public static final double climberElevatorHighThreshold = 0;

    private static final double diffectorGearTeethIn = 10;
    private static final double diffectorGearTeethOut = 60;
    private static final double diffectorSprocketTeethIn  = 18;
    private static final double diffectorSprocketTeethOut = 72;
    /** Output sprocket degrees per motor rotation */
    public static final double gearboxRatio = 1.25 / (diffectorGearTeethIn / diffectorGearTeethOut); // I have ABSOLUTELY NO IDEA where the 1.25 scale comes from
    /** Ratio of output sprocket to arm sprocket (output sprocket teeth/arm sprocket teeth) */
    public static final double sprocketRatio = (diffectorSprocketTeethIn / diffectorSprocketTeethOut);
    /** Pitch Diameter of the sprocket, in m */
    public static final double sprocketPitchDiameter = 0.036576;

    /** 
     * Metres of chain moved per sprocket degree.
     */
    public static final double travelRatio = (sprocketPitchDiameter * Math.PI) / 360;
    /** 
     * Number of arm degrees moved for one motor degree of a single motor 
     * Output sprocket rotations per motor rotation * output sprocket to arm sprocket ratio,
     * divided by 2 to give the contribution of a single motor
     */
    public static final double rotationRatio = (sprocketRatio);

    public static final boolean startingCoralState = true;
    public static final boolean startingAlgaeState = false;

    public static final double maxRotation = 1;
    /** Maximum total angle the arm is allowed to rotate away from centre */
    public static final double maxAbsAngle = maxRotation * 360;
    /** Above this angle, the arm can turn towards centre even if it's a longer path */
    public static final double turnBackThreshold = 135;

    /** Physical upper limit of the elevator, metres above the ground */
    public static final double maxElevation = 1.709;
    /** Physical lower limit of the elevator, metres above the ground */
    public static final double minElevation = 0.444;
    
    /** Arm rotation check tollerance, degrees */
    public static final double angleTolerance = 2;
    
    /** Elevation height check tolerance, m */
    public static final double elevationTolerance = 0.025;
    
    /* Preset arm angles, degrees anticlockwise for Port-side usecase, 0 = coral at top */
    public static final double startAngle         =   0;
    public static final double climbAngle         =  90;
    public static final double netAngle           = 225;
    public static final double processorAngle     = 270;
    public static final double reef4Angle         = 45;
    public static final double reef3Angle         = 30;
    public static final double reef2Angle         = 30;
    public static final double reef1Angle         = 135;
    public static final double coralTransferAngle = 180; 
    public static final double algaeTransferAngle =   0;
    public static final double coralStationAngle  = 240; //TODO
    public static final double algae2Angle        = 270;
    public static final double algae1Angle        = 270; 
    public static final double algaeStowAngle     = 180; 
    public static final double coralStowAngle     = 0;
    
    /* Preset elevator heights, height of centre of rotation above the ground, metres */
    public static final double startElevation         = 0.574; 
    public static final double climbElevation         = minElevation;
    public static final double netElevation           = maxElevation;
    public static final double processorElevation     = minElevation; 
    public static final double reef4Elevation         = maxElevation;
    public static final double reef3Elevation         = 1.28;
    public static final double reef2Elevation         = 0.9;
    public static final double reef1Elevation         = 0.8;
    public static final double coralTransferElevation = 1.2; //TODO
    public static final double algaeTransferElevation = 1.2; //TODO
    public static final double coralStationElevation  = 1.0; //TODO
    public static final double algae2Elevation        = 1.13;
    public static final double algae1Elevation        = 0.92;
    public static final double algaeStowElevation     = 0.8; //TODO
    public static final double coralStowElevation     = 0.8; //TODO
    
    public static final class IKGeometry
    {
      /* Manipulator arm geometry */
      public static final double coralArmLength   = 0.5;
      public static final double coralArmAngle    = 64/2;
      public static final double algaeArmLength   = 0.6;
      public static final double algaeArmAngle    = 60/2;
      public static final double algaeWheelLength = 0.54;
      public static final double algaeClawLength  = 0.48;
      public static final double algaeInnerLength = 0.3;
      public static final double algaeInnerAngle  = 108/2;

      /* Deck obstruction geometry */
      public static final double railHeight    = 0.23;
      public static final double railLateral   = 0.38;
      public static final double railMedial    = 0.27;
      public static final double deckHeight    = 0.18;
      public static final double harpoonHeight = 0.05;
      public static final double harpoonAngle  = 2;

      /** For IK, angle the arm is projected to test for immediate collisions, degrees */
      public static final double projectionAngle = 5;
      /** For IK, distance the arm is projected down to test for immediate collisions, m */
      public static final double projectionElevation = 0.1;

      /** For pathfollowing, elevation/rotation "distance" to set the dynamic target position at */
      public static final Translation2d unitTravel = new Translation2d(projectionElevation, projectionAngle);
    }
  }

  public static final class GamePiecesManipulator 
  {
    /* Coral manipulator speeds */
    public static final double coralManipulatorBaseIntakeSpeed = 0.2;
    public static final double coralManipulatorMaxIntakeSpeed  = 0.4;
    public static final double coralManipulatorDeliverySpeed   = 0.7;
    public static final double coralManipulatorHoldingVoltage  = 0;

    /* Algae manipulator speeds */
    public static final double algaeManipulatorIntakeSpeed    = -0.4;
    public static final double algaeManipulatorHoldingVoltage = 0;
    public static final double algaeManipulatorNetSpeed       = 0.9;
    public static final double algaeManipulatorProcessorSpeed = 0.5;
    public static final double algaeManipulatorEmptySpeed     = 0;

    /* The number of cycles (each cycle is 0.02s) before the holding intake speed reaches max */
    public static final double coralHoldingScalar = 100;
  }

  public static final class IntakeConstants // TODO: Speeds and Angles must be tuned to the specific robot
  {
    /* Intake motors speeds */
    public static final double algaeIntakeMotorSpeed = 0.8;
    public static final double algaeEjectMotorSpeed = -0.8;
    public static final double climbingIntakeMotorSpeed = 0;
    public static final double standByMotorSpeed = 0;
    public static final double stowedMotorSpeed = 0;
    public static final double algaeTransferMotorSpeed = 0;

    /* Top intake arm positions 
     * TODO: Put in Degrees for the arm top and bottom position in this comment
     */
    public static final double topAlgaeIntakeArmTarget   = 0;
    public static final double topAlgaeEjectArmTarget    = 0;
    public static final double algaeClimbingArmTarget    = 0;
    public static final double topStandByArmTarget       = 0;
    public static final double topStowedArmTarget        = 0;
    public static final double topAlgaeTransferArmTarget = 0;

    public static final double algaeStowedLowThreshold  = 10;
    public static final double algaeStowedHighThreshold = 10;

    /* Top arm PID + FeedForward values */
    public static final double topArmSpringKP = 1;
    public static final double topArmSpringKI = 0;
    public static final double topArmSpringKD = 0;
    public static final double topArmStopKP   = 12.5;
    public static final double topArmStopKI   = 0;
    public static final double topArmStopKD   = 0;
    
    public static final double topArmKS = 0.15;
    public static final double topArmKG = 0.15;

    /* Arm MotionMagic values */
    public static final double intakeArmMotionMagicCruise = 0.25;
    public static final double intakeArmMotionMagicAccel  = 0.25;

    /* Arm ratios */
    public static final double topArmRatio    = 16.7;
  }

  public static final class ClimberConstants
  {
    public static final double initSpeed   = 0;
    public static final double deploySpeed = -0.5;
    public static final double climbSpeed  = 0.8;

    public static final double initWinchPos   = 0;
    public static final double deployWinchPos = 0;
    public static final double climbWinchPos  = 0;

    public static final double winchKP = 0;
    public static final double winchKI = 0;
    public static final double winchKD = 0;

    public static final double winchMotionMagicCruise = 0;
    public static final double winchMotionMagicAccel  = 0;

    public static final double initWinchThreshold = 10;
  }
}
