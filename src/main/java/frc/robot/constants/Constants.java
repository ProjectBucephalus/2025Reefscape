package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.function.Supplier;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.util.AutoUtils;
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
    public static final double maxThrottle = 0.7;
    /** Minimum robot speed when braking, relative to maximum uncapped speed */
    public static final double minThrottle = 0.1;
    /** Normal maximum rotational robot speed, relative to maximum uncapped rotational speed */
    public static final double maxRotThrottle = 1;
    /** Minimum rotational robot speed when braking, relative to maximum uncapped rotational speed */
    public static final double minRotThrottle = 0.5;
    /** Angle tolerance to consider something as "facing" the drivers, degrees */
    public static final double driverVisionTolerance = 5;
    /** Scalar for manual diffector elevation control */
    public static final double manualDiffectorElevationScalar = 2;
    /** Scalar for manual diffector rotation control */
    public static final double manualDiffectorRotationScalar = 2;
    /** Scalar for braking effect of diffector arm being higher than 1m */
    public static final double armBrakeRate = 1.5;
  }

  public static final class Vision
  {
    /* public static final int[] validIDs = 
    {
      //1, 2, 3,               // Red Human Player Stations
      //4, 5,                  // Red Barge
      //6, 7, 8, 9, 10, 11,      // Red Reef
      //12, 13, 16,            // Blue Human Player Stations
      //14, 15,                // Blue Barge
      17, 18, 19, 20, 21, 22   // Blue Reef
    };*/

    public static final int[] reefIDs = 
    {
      6, 7, 8, 9, 10, 11,    // Red Reef
      17, 18, 19, 20, 21, 22 // Blue Reef
    };

    public static final int[] bargeIDs = 
    {
      4, 5,  // Red Barge
      14, 15 // Blue Barge
    };

    public static final int[] humanPlayerStationIDs = 
    {
      //1, 2, 3,   // Red Human Player Stations
      12, 13, 16 // Blue Human Player Stations
    };

    /** Baseline 1 meter, 1 tag stddev for x and y, in meters */
    public static final double linearStdDevBaseline = 0.06;
    /** Baseline 1 meter, 1 tag stddev rotation, in radians */
    public static final double rotStdDevBaseline = 0.012;
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
    public static final double rotationKP = 5; //TODO: Tune to robot
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
    public static final double pathplannerMaxSpeed = 10;
    /** m/s */
    public static final double pathplannerSlowedSpeed = 0.1;
    /** m/s^2 */
    public static final double pathplannerMaxAcceleration = 3;
    /** degrees/s */
    public static final double pathplannerMaxAngularSpeed = 720;
    /** degrees/s^2 */
    public static final double pathplannerMaxAngularAcceleration = 1050;
    public static final PathConstraints defaultConstraints = new PathConstraints
      (pathplannerMaxSpeed, pathplannerMaxAcceleration, pathplannerMaxAngularSpeed, pathplannerMaxAngularAcceleration);
    
    public static final PathConstraints slowedConstraints = new PathConstraints
      (pathplannerSlowedSpeed, pathplannerMaxAcceleration, pathplannerMaxAngularSpeed, pathplannerMaxAngularAcceleration);
    
    public static final Map<Translation2d, Integer> reefMidPointMap = new HashMap<>(6)
    {{
      put(new Translation2d(3.658, 4.026), 1);
      put(new Translation2d(4.073, 3.306), 2);
      put(new Translation2d(4.905, 3.306), 3);
      put(new Translation2d(5.321, 4.026), 4);
      put(new Translation2d(4.905, 4.746), 5);
      put(new Translation2d(4.073, 4.746), 6);
    }};

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
    {{
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
      put("cl1", new AutoMapping("cl1", null));
      put("cl2", new AutoMapping("cl2", null));
      put("cl3", new AutoMapping("cl3", null));
      put("cr1", new AutoMapping("cr1", null));
      put("cr2", new AutoMapping("cr2", null));
      put("cr3", new AutoMapping("cr3", null));
      put("a1" , new AutoMapping("a1" , () -> AutoUtils.intakeAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, 1)));
      put("a2" , new AutoMapping("a2" , () -> AutoUtils.intakeAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, 2)));                
      put("a3" , new AutoMapping("a3" , () -> AutoUtils.intakeAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, 3)));                
      put("a4" , new AutoMapping("a4" , () -> AutoUtils.intakeAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, 4)));                
      put("a5" , new AutoMapping("a5" , () -> AutoUtils.intakeAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, 5)));                
      put("a6" , new AutoMapping("a6" , () -> AutoUtils.intakeAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, 6)));
      put("b1" , new AutoMapping("b1" , () -> AutoUtils.scoreAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, true)));
      put("b2" , new AutoMapping("b3" , () -> AutoUtils.scoreAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, true)));
      put("b3" , new AutoMapping("b5" , () -> AutoUtils.scoreAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, true)));
      put("p"  , new AutoMapping("p"  , () -> AutoUtils.scoreAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, false)));                
      put("kl" , new AutoMapping("kl", null));
      put("kr" , new AutoMapping("kr", null));
      put("e"  , new AutoMapping(null, () -> Commands.defer(() -> AutoUtils.ejectAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, () -> RobotContainer.swerveState.Pose.getTranslation()), new HashSet<Subsystem>(){{add(RobotContainer.s_Algae); add(RobotContainer.s_Diffector);}})));
    }};

    public static final ArrayList<Translation2d> reefBlueMidPoints = FieldUtils.GeoFencing.reefBlue.getMidPoints();
    public static final ArrayList<Translation2d> reefRedMidPoints = FieldUtils.GeoFencing.reefRed.getMidPoints();

    public static final ArrayList<Translation2d> blueBargePoints = new ArrayList<Translation2d>()
    {{
      add(new Translation2d(FieldUtils.fieldLength / 2, 7.261));
      add(new Translation2d(FieldUtils.fieldLength / 2, 6.615));
      add(new Translation2d(FieldUtils.fieldLength / 2, 6.169));
      add(new Translation2d(FieldUtils.fieldLength / 2, 5.6245));
      add(new Translation2d(FieldUtils.fieldLength / 2, 5.08));
    }};

    public static final ArrayList<Translation2d> redBargePoints = new ArrayList<Translation2d>(blueBargePoints)
    {
      {
        forEach(point -> point.rotateAround(new Translation2d(FieldUtils.fieldLength / 2, FieldUtils.fieldWidth / 2), Rotation2d.k180deg));
      }
    };

    /** How close we have to be to the path start point to just follow the path without using pathfinding */
    public static final double pathFollowTolerance = 0.04;

    public static final String defaultAuto = "rc4,cr1,rb4,cl1,ra4,cl1,rl4";
  }

  public static final class DiffectorConstants
  {
    public static final double motorStallCurrent = 100; // TODO: Tune this to the point that it will reliably prevent stalls

    public static final double diffectorMotorKG = 0.225;
    public static final double diffectorMotorKS = 0.05;
    public static final double diffectorMotorKV = 0.58;
    public static final double diffectorMotorKP = 100;
    public static final double diffectorMotorKI = 0;
    public static final double diffectorMotorKD = 0;

    public static final double diffectorMotorKGSpring = 0;
    public static final double diffectorMotorKSSpring = 0;
    public static final double diffectorMotorKVSpring = 0;
    public static final double diffectorMotorKPSpring = 3;
    public static final double diffectorMotorKISpring = 0;
    public static final double diffectorMotorKDSpring = 0;

    
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
    
    /** Desired cruise speed of Motor, RPS */
    public static final double diffectorCruiseMotor = 90;
    /** Desired cruise speed of Mechanism, RPS */
    public static final double diffectorCruise = diffectorCruiseMotor / gearboxRatio;
    /** Desired acceleration of Motor, RPS^2 */
    public static final double diffectorElevationAccelerationMotor = 70;
    /** Desired acceleration of Mechanism, RPS^2 */
    public static final double diffectorElevationAcceleration = diffectorElevationAccelerationMotor / gearboxRatio;
    /** Desired acceleration of Motor, RPS^2 */
    public static final double diffectorRotationAccelerationMotor = 70;
    /** Desired acceleration of Mechanism, RPS^2 */
    public static final double diffectorRotationAcceleration = diffectorRotationAccelerationMotor / gearboxRatio;
    
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
    public static final double minZ = 0.36;
    /** Elevation at which all rotations are safe */
    public static final double safeElevation = 0.71;
    public static final double reefSafeElevation = 1; // TODO
    public static final double algaeSafeElevation = 1;
    public static final double climberClearanceThreshold = 0.6; // TODO
    
    /** Arm rotation check tollerance, degrees */
    public static final double angleTolerance = 2;
    
    /** Elevation height check tolerance, m */
    public static final double elevationTolerance = 0.01;

    /** Number of clock cycles delay before arm is calibrated after reaching a target */
    public static final int calibrationDelay = 30;

    public static final int algaeEjectSpeedAngleThreshold = 30;
    
    /** 
     * Preset arm positions:
     * height of centre of rotation above the ground, metres
     * degrees anticlockwise for Port-side usecase, 0 = coral at top 
     */
    public static class Presets
    {
      public static final Translation2d startPosition           = new Translation2d(0.616,  0);
      public static final Translation2d climbSafePosition       = new Translation2d(0.70,  90);
      public static final Translation2d climbPosition           = new Translation2d(0.425, 90);
 
      public static final Translation2d netPosition             = new Translation2d(  maxZ, 170);
      public static final Translation2d algae3PortPosition      = new Translation2d(1.08, 113);
      public static final Translation2d algae3StbdPosition      = new Translation2d(1.08, 247);
      public static final Translation2d algae2PortPosition      = new Translation2d(0.65, 113);
      public static final Translation2d algae2StbdPosition      = new Translation2d(0.65, 247);
      public static final Translation2d processorPosition       = new Translation2d(0.43,  90);
 
      public static final Translation2d coral4PortPosition      = new Translation2d(1.62, 340);
      public static final Translation2d coral4StbdPosition      = new Translation2d(1.62,  20);
      public static final Translation2d coral3PortPosition      = new Translation2d(0.98, 340);
      public static final Translation2d coral3StbdPosition      = new Translation2d(0.98,  20);
      public static final Translation2d coral2PortPosition      = new Translation2d(0.68, 325);
      public static final Translation2d coral2StbdPosition      = new Translation2d(0.68,  35);
      public static final Translation2d coral1ClawPortPosition  = new Translation2d(0.71,  90);
      public static final Translation2d coral1ClawStbdPosition  = new Translation2d(0.71, 270);
      public static final Translation2d coral1PortPosition      = new Translation2d(0.78, 150);
      public static final Translation2d coral1StbdPosition      = new Translation2d(0.78, 210);
 
      public static final Translation2d coralIntakePortPosition = new Translation2d(1.15, 215);
      public static final Translation2d coralIntakeStbdPosition = new Translation2d(1.15, 145);
      public static final Translation2d coralClawPortPosition   = new Translation2d(0.61, 133);
      public static final Translation2d coralClawStbdPosition   = new Translation2d(0.61, 227);
      public static final Translation2d coralStowPosition       = new Translation2d(0.70,   0);
 
      public static final Translation2d algaeIntakePortPosition = new Translation2d(0.38,  75);
      public static final Translation2d algaeIntakeStbdPosition = new Translation2d(0.38, 285);
      public static final Translation2d algaeStowPosition       = new Translation2d(0.65, 180);

      public static final ArrayList<Translation2d> lowDiffectorPositions = new ArrayList<Translation2d>()
      {{
        add(startPosition);
        add(algaeIntakePortPosition);
        add(algaeIntakeStbdPosition);
        add(climbPosition);
        add(processorPosition);
      }};
    }
        
    public static final class IKGeometry
    {
      /** Manipulator arm point-cloud */
      public static final Translation2d[] armGeometry = new Translation2d[]
      {
        new Translation2d(0.275, 0.295),
        new Translation2d(0.275, 0.445),
        new Translation2d(0.000, 0.490),
        new Translation2d(-0.275, 0.445),
        new Translation2d(-0.275, 0.295),
        new Translation2d(-0.110, -0.545),
        new Translation2d(0.000, -0.555),
        new Translation2d(0.110, -0.545)
      };
      /** Manipulator arm point-cloud when holding Algae */
      public static final Translation2d[] armGeometryAlgae = new Translation2d[]
      {
        new Translation2d(0.275, 0.295),
        new Translation2d(0.275, 0.445),
        new Translation2d(0.000, 0.490),
        new Translation2d(-0.275, 0.445),
        new Translation2d(-0.275, 0.295),
        new Translation2d(0.216, -0.646),
        new Translation2d(0.187, -0.753),
        new Translation2d(0.108, -0.833),
        new Translation2d(0.000, -0.861),
        new Translation2d(-0.108, -0.833),
        new Translation2d(-0.187, -0.753),
        new Translation2d(-0.216, -0.646)
      };

      /* Deck obstruction geometry */
      public static final double deckHeight  = 0.16;
      public static final double railHeight  = 0.2;
      public static final double railLateral = 0.45;
      public static final double railMedial  = 0.37;

      /** For IK, angle the arm is projected to test for immediate collisions, degrees */
      public static final double projectionAngle = 5;
      /** For IK, distance the arm is projected down to test for immediate collisions, m */
      public static final double projectionElevation = 0.05;

      /** For pathfollowing, elevation/rotation "distance" to set the dynamic target position at */
      public static final Translation2d unitTravel = new Translation2d(projectionElevation, projectionAngle);

      public static final double reefSafetyRadius = 1.7;

      /** Distance from centre of barge where arm height needs to be checked, metres */
      public static final double bargeSafetyWidth = 0.85;
      /** Minimum height over ground where arm height needs to be checked, metres */
      public static final double bargeSafetyHeight = 1;
    }

    /** Raw value when fully released, indicating string has snapped or the sensor is unavailable */
    public static final double potErrValue = 1;
    /** Elevator height when potentiometer reads 0, metres over ground */
    public static final double potMin = 0.0;
    /** Elevator height when potentiometer reads 1, metres over ground */
    public static final double potMax = 2.0;
  }

  public static final class GamePiecesManipulator 
  {
    /* Coral manipulator speeds */
    public static final double coralDeliverySpeed = -0.7;
    public static final double coralHoldingSpeed  = -0.05;
    public static final double coralHoldingkG = -0.035;

    /* Algae manipulator speeds */
    public static final double algaeIntakeSpeed    =  0.4;
    public static final double algaeHoldingSpeed   =  0.2;
    public static final double algaeNetSpeed       = -0.9;
    public static final double algaeProcessorSpeed = -0.3;
    public static final double algaeHeldCurrent = 30;

    /** Algae net shooting range for rotation snapping, m */
    public static final double algaeRange = 2.5;
    /** How far towards the barge we have to be from field center to be able to score in the net (Y axis) */
    public static final double netScoringCenterDistance = 0.5;
    /** Target X distance from barge targetting points for scoring */
    public static final double netScoringOffset = 1.5;
  }

  public static final class ClimberConstants
  {
    public static final double stowWinchPos = 0;
    public static final double activeWinchPos = 1.5;
    public static final double climbWinchPos  = 0.75;
    /** The furthest into the robot the climber can attempt to go whilst balancing */
    public static final double climbActiveInnerLimit = 0.65;
    /** The furthest out of the robot the climber can attempt to go whilst balancing */
    public static final double climbActiveOuterLimit = 0.85;
    public static final double manualScale    = 0.25;

    public static final double winchKP = 150;
    public static final double winchKI = 0;
    public static final double winchKD = 0;
    public static final double winchBalanceScalar = 0.05;

    public static final double winchPlanetaryRatio = 45;
    public static final double winchGearIn = 20;
    public static final double winchGearOut = 60;
    public static final double winchGearRatio = ((winchGearOut / winchGearIn) * winchPlanetaryRatio);
    public static final double winchDefaultCruise = 1;
    public static final double winchClimbCruise = 0.5;
    public static final double winchMotionMagicAccel  = 1;
  }

  public static final class LEDStrip
  {
    /**
     * Constants used by the addressable LED classes.
     */
    public static final int LEDPWMPort = IDConstants.LEDPWM; // PWM port the strip is connected to.
    public static final int lightsLen = 120; // # of LED's in the strip, if more than one strip daisy-chained, total # of LED's
    public static final int viewWidth = 30; // default width for a partial display layer. a good number is about 1/4 lightsLen
    public static final int startOffset = 1; // LED # at 0 degrees
    public static final double degreesPerLED = 360/lightsLen;  // used to calculate the LED pointing in a particular direction
    public static final Color defaultBackColor = Color.kRed; // default background / off Color
    public static final Color defaultFrontColor = Color.kGreen; // default foreground / on Color
    public static final Color displayBorderColor = Color.kBlue; // default Color for border dots
    public static final int defaultStatusSegments = 3; // default # of segments in a status display
    public static final int pointerGradientThreshold = 5; // length of pointer layer above which a gradient will be applied rather than solid colour
    public static final int stbdLEDsStart = 0; // start and end LED #'s for the 'starboard' segment
    public static final int stbdLEDsEnd = 59;
    public static final int portLEDsStart = 60; // start and end LED #'s for the 'port' segment
    public static final int portLEDsEnd = 119;
    public static final int discoMax = 10;  // maximum colour layers in disco mode
    public static final int discoMin = 3;  // minimum colour layers in disco mode
    public static final int discoAgeLimit = 10; //disco layers will randomly die of age between discoAgeLimit and 2x discoAgeLimit cycles
    public static final double discoChangeChance = 0.9; // the probability of accel or growthrate changing in any update is 1 - this: (1 - 0.9 = 0.1 = 10% chance of change)
    public static final double discoMaxVelMultiplier = 0.3; // used to calculate maxVel based on viewWidth, at maximum velocity it will take 1/this seconds to traverse the strip.
    public static final double discoMaxAccelMultiplier = 0.1; // used to calculate maxAccel, maxVel will be multiplied by this to get the value.
    public static final double discoMaxGrowMultiplier = 0.1; // used to calculate maxGrow based on viewWidth.
    public static final double discoMaxGrowRateMultiplier = 0.1; // multipied by maxGrow to get maxGrowRate
    public static final double discoColorThreshold = 0.5; // at least one color value (r,g,b) must be above this for the colour to be valid
  }
}
