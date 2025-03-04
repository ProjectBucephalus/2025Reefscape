package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.Manipulator.IntakeAlgaeSequence;
import frc.robot.commands.Manipulator.IntakeCoralSequence;
import frc.robot.commands.Manipulator.ScoreAlgaeSequence;
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
    public static final double manualDiffectorDeadband = 0.25;
    public static final double stickDeadband = 0.15;
    /** Normal maximum robot speed, relative to maximum uncapped speed */
    public static final double maxThrottle = 0.6;
    /** Minimum robot speed when braking, relative to maximum uncapped speed */
    public static final double minThrottle = 0.1;
    /** Normal maximum rotational robot speed, relative to maximum uncapped rotational speed */
    public static final double maxRotThrottle = 1;
    /** Minimum rotational robot speed when braking, relative to maximum uncapped rotational speed */
    public static final double minRotThrottle = 0.5;
    /** Angle tolerance to consider something as "facing" the drivers, degrees */
    public static final double driverVisionTolerance = 5;
    /** Scalar for manual diffector control */
    public static final double manualDiffectorScalar = 2;
  }

  public static final class Vision
  {
    public static final int[] validIDs = 
    {
      //1, 2, 3,               // Red Human Player Stations
      //4, 5,                  // Red Barge
      //6, 7, 8, 9, 10, 11,      // Red Reef
      //12, 13, 16,            // Blue Human Player Stations
      //14, 15,                // Blue Barge
      17, 18, 19, 20, 21, 22   // Blue Reef
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
        put("cr1", new AutoMapping("cr1", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr2", new AutoMapping("cr2", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("cr3", new AutoMapping("cr3", () -> new IntakeCoralSequence(RobotContainer.s_Diffector, RobotContainer.s_CoralManipulator)));
        put("a1" , new AutoMapping("a1" , () -> new IntakeAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator, () -> RobotContainer.swerveState.Pose.getTranslation())));
        put("a2" , new AutoMapping("a2" , () -> new IntakeAlgaeSequence(false, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator, () -> RobotContainer.swerveState.Pose.getTranslation())));                
        put("a3" , new AutoMapping("a3" , () -> new IntakeAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator, () -> RobotContainer.swerveState.Pose.getTranslation())));                
        put("a4" , new AutoMapping("a4" , () -> new IntakeAlgaeSequence(false, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator, () -> RobotContainer.swerveState.Pose.getTranslation())));                
        put("a5" , new AutoMapping("a5" , () -> new IntakeAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator, () -> RobotContainer.swerveState.Pose.getTranslation())));                
        put("a6" , new AutoMapping("a6" , () -> new IntakeAlgaeSequence(false, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator, () -> RobotContainer.swerveState.Pose.getTranslation())));
        put("b1" , new AutoMapping("b1" , () -> new ScoreAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));
        put("b2" , new AutoMapping("b3" , () -> new ScoreAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));
        put("b3" , new AutoMapping("b5" , () -> new ScoreAlgaeSequence(true, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));
        put("p"  , new AutoMapping("p"  , () -> new ScoreAlgaeSequence(false, RobotContainer.s_Diffector, RobotContainer.s_AlgaeManipulator)));                
      }
    };

    public static final ArrayList<Translation2d> reefBlueMidPoints = FieldUtils.GeoFencing.reefBlue.getMidPoints();
    public static final ArrayList<Translation2d> reefRedMidPoints = FieldUtils.GeoFencing.reefRed.getMidPoints();

    public static final ArrayList<Translation2d> blueBargePoints = new ArrayList<Translation2d>()
    {
      {
        add(new Translation2d(FieldUtils.fieldLength / 2, 5.08));
        add(new Translation2d(FieldUtils.fieldLength / 2, 5.6245));
        add(new Translation2d(FieldUtils.fieldLength / 2, 6.169));
        add(new Translation2d(FieldUtils.fieldLength / 2, 6.615));
        add(new Translation2d(FieldUtils.fieldLength / 2, 7.261));
      }
    };

    public static final ArrayList<Translation2d> redBargePoints = new ArrayList<Translation2d>(blueBargePoints)
    {
      {
        forEach(point -> point.rotateAround(new Translation2d(FieldUtils.fieldLength / 2, FieldUtils.fieldWidth / 2), Rotation2d.k180deg));
      }
    };


    public static final String defaultAuto = "rc4,cr1,rb4,cl1,ra4,cl1,rl4";
  }

  public static final class DiffectorConstants
  {
    public static final double motorStallCurrent = 100; // TODO: Tune this to the point that it will reliably prevent stalls

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
    public static final double diffectorMotionMagicCruise = 90;
    /** Desired acceleration OF MOTOR, RPS^2 */
    public static final double diffectorMotionMagicAccel  = 45;

    public static final double coralElevatorLowTheshold = 0;
    public static final double coralElevatorHighThreshold = 0;
    public static final double algaeElevatorLowTheshold = 0;
    public static final double algaeElevatorHighThreshold = 0;
    public static final double climberElevatorLowTheshold = 0;
    public static final double climberElevatorHighThreshold = 0;

    private static final double diffectorGearTeethIn = 8;
    private static final double diffectorGearTeethOut = 60;
    private static final double diffectorSprocketTeethIn  = 18;
    private static final double diffectorSprocketTeethOut = 72;
    /** Output sprocket degrees per motor rotation */
    public static final double gearboxRatio = (diffectorGearTeethOut / diffectorGearTeethIn);
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

    public static final double maxRotation = 5;
    /** Maximum total angle the arm is allowed to rotate away from centre */
    public static final double maxAbsAngle = maxRotation * 360;
    /** Above this angle, the arm can turn towards centre even if it's a longer path */
    public static final double turnBackThreshold = 135;

    /** Physical upper limit of the elevator, metres above the ground */
    public static final double maxZ = 1.725;
    /** Physical lower limit of the elevator when horizontal, metres above the ground */
    public static final double minZ = 0.42;
    /** Elevation at which all rotations are safe */
    public static final double safeElevation = 0.8;
    public static final double reefSafeElevation = 1;
    
    /** Arm rotation check tollerance, degrees */
    public static final double angleTolerance = 2;
    
    /** Elevation height check tolerance, m */
    public static final double elevationTolerance = 0.01;

    public static final int algaeEjectSpeedAngleThreshold = 30;
    
    /* 
     * Preset arm positions:
     * height of centre of rotation above the ground, metres
     * degrees anticlockwise for Port-side usecase, 0 = coral at top 
     */
    public static final Translation2d startPosition         = new Translation2d(0.57,   0);
    public static final Translation2d climbPosition         = new Translation2d(0.42,  90);
    public static final Translation2d netPosition           = new Translation2d(  maxZ, 170);
    public static final Translation2d algae3Position        = new Translation2d(1.32, 275);
    public static final Translation2d algae2Position        = new Translation2d(0.92, 275); 
    public static final Translation2d algae3FlippedPosition = new Translation2d(1.32,  -275); 
    public static final Translation2d algae2FlippedPosition = new Translation2d(0.92,  96.5);
    public static final Translation2d processorPosition     = new Translation2d(0.44,  96);
    public static final Translation2d reef4Position         = new Translation2d(  maxZ,  35);
    public static final Translation2d reef3Position         = new Translation2d(1.05,  30);
    public static final Translation2d reef2Position         = new Translation2d(0.72,  45); //TODO
    public static final Translation2d reef1Position         = new Translation2d(0.5,   40); //TODO
    public static final Translation2d reef4FlippedPosition  = new Translation2d(  maxZ,   -35);
    public static final Translation2d reef3FlippedPosition  = new Translation2d(1.05,   -30); 
    public static final Translation2d reef2FlippedPosition  = new Translation2d(0.72,   -45); //TODO
    public static final Translation2d reef1FlippedPosition  = new Translation2d(0.5,    -40); //TODO
    public static final Translation2d coralTransferPosition = new Translation2d(0.60, 180); 
    public static final Translation2d coralIntakePosition   = new Translation2d(0.90,  90);
    public static final Translation2d algaeTransferPosition = new Translation2d(0.90,   0);
    public static final Translation2d algaeIntakePosition   = new Translation2d(0.48,  60);
    public static final Translation2d algaeStowPosition     = new Translation2d(0.80, 180); 
    public static final Translation2d coralStowPosition     = new Translation2d(0.80,   0);
        
    public static final class IKGeometry
    {
      /* Manipulator arm geometry */
      public static final Translation2d[] armGeometry = new Translation2d[]
      {
        new Translation2d(0.16,0.42),
        new Translation2d(0.16,0.48),
        new Translation2d(0.14,0.48),
        new Translation2d(0.12,0.48),
        new Translation2d(0.00,0.48),
        new Translation2d(-0.12,0.48),
        new Translation2d(-0.14,0.48),
        new Translation2d(-0.16,0.48),
        new Translation2d(-0.16,0.42),
        new Translation2d(-0.24,-0.35),
        new Translation2d(-0.24,-0.37),
        new Translation2d(-0.24,-0.40),
        new Translation2d(-0.24,-0.45),
        new Translation2d(-0.23,-0.46),
        new Translation2d(-0.22,-0.47),
        new Translation2d(-0.20,-0.47),
        new Translation2d(-0.18,-0.46),
        new Translation2d(0.12,-0.50),
        new Translation2d(0.12,-0.55),
        new Translation2d(0.15,-0.58),
        new Translation2d(0.20,-0.58),
        new Translation2d(0.23,-0.55),
        new Translation2d(0.23,-0.53),
        new Translation2d(0.24,-0.50),
        new Translation2d(0.24,-0.45),
        new Translation2d(0.24,-0.40),
        new Translation2d(0.24,-0.37),
        new Translation2d(0.24,-0.35)
      };

      /* Deck obstruction geometry */
      public static final double railHeight  = 0.2;
      public static final double railLateral = 0.45;
      public static final double railMedial  = 0.37;
      public static final double deckHeight  = 0.165;

      /** For IK, angle the arm is projected to test for immediate collisions, degrees */
      public static final double projectionAngle = 5;
      /** For IK, distance the arm is projected down to test for immediate collisions, m */
      public static final double projectionElevation = 0.1;

      /** For pathfollowing, elevation/rotation "distance" to set the dynamic target position at */
      public static final Translation2d unitTravel = new Translation2d(projectionElevation, projectionAngle);

      public static final double reefSafetyRadius = 1.7;
    }
  }

  public static final class GamePiecesManipulator 
  {
    /* Coral manipulator speeds */
    public static final double coralManipulatorDeliverySpeed   = -0.7;
    public static final double coralManipulatorHoldingSpeed  = -0.05;
    public static final double coralHoldingkG = -0.035;

    /* Algae manipulator speeds */
    public static final double algaeManipulatorIntakeSpeed    = 0.4;
    public static final double algaeManipulatorNetSpeed       = -0.9;
    public static final double algaeManipulatorProcessorSpeed = -0.6;

    /** Algae net shooting range for rotation snapping, m */
    public static final double algaeRange = 1.3;
    /** How far towards the barge we have to be from field center to be able to score in the net (Y axis) */
    public static final double netScoringCenterDistance = 0.5;
    /** Target X distance from barge targetting points for scoring */
    public static final double netScoringOffset = 1.5;
  }

  public static final class ClimberConstants
  {
    public static final double lockedWinchPos = 0;
    public static final double activeWinchPos = 1.85;
    public static final double climbWinchPos  = -0.3;
    public static final double intakeWinchPos = 0.2;

    public static final double winchKP = 1;
    public static final double winchKI = 0;
    public static final double winchKD = 0;

    public static final double winchPlanetaryRatio = 45;
    public static final double winchGearIn = 20;
    public static final double winchGearOut = 60;
    public static final double winchGearRatio = -(winchGearOut / winchGearIn) * winchPlanetaryRatio;
    public static final double winchDefaultCruise = 10;
    public static final double winchClimbCruise = 10;
    public static final double winchMotionMagicAccel  = 10;
  }
}
