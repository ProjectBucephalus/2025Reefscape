package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.util.ArmPos;
import frc.robot.util.AutoUtils;

public final class Constants 
{
  public static final class RumblerConstants 
  {
    public static final double driverDefault = 0.1;
    public static final double copilotDefault = 0.1;
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
    public static final double manualClimberScale = 1;
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
      1, 2, 3,   // Red Human Player Stations
      12, 13, 16 // Blue Human Player Stations
    };

    /** Baseline 1 meter, 1 tag stddev for x and y, in meters */
    public static final double linearStdDevBaseline = 0.08;
    /** Baseline 1 meter, 1 tag stddev rotation, in radians */
    public static final double rotStdDevBaseline = 999;
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
    public static final double rotationKP = 6;
    public static final double rotationKI = 0;
    public static final double rotationKD = 0;
    
    /* Rotation Control PID Values when holding Algae */
    public static final double rotationKPAlgae = 5;
    public static final double rotationKIAlgae = 0;
    public static final double rotationKDAlgae = 1;

    /* Swerve Limit Values */
    /** Meters per Second */
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Radians per Second */
    public static final double maxAngularVelocity = 4;
  }

  public static final class Auto
  {   
    /** m/s */
    public static final double pathplannerMaxSpeed = 4.35;
    /** m/s */
    public static final double pathplannerSlowedSpeed = 1.5;
    /** m/s^2 */
    public static final double pathplannerMaxAcceleration = 4.0;
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
      put("e"  , new AutoMapping(null, () -> Commands.defer(() -> AutoUtils.ejectAlgaeSequenceCommand(RobotContainer.s_Diffector, RobotContainer.s_Algae, () -> RobotContainer.swerveState.Pose.getTranslation()), Set.of(RobotContainer.s_Algae, RobotContainer.s_Diffector))));
    }};

    /** How close we have to be to the path start point to just follow the path without using pathfinding */
    public static final double atPosTolerance = 0.07;

    public static final String defaultAuto = "rc4,cr1,rb4,cl1,ra4,cl1,rl4";
  }

  public static final class DiffectorConstants
  {
    public static final boolean startingCoralState = true;
    public static final boolean startingAlgaeState = true;

    /** Number of clock cycles delay before arm is calibrated after reaching a target */
    public static final int calibrationDelay = 30;

    /** 
     * Preset arm positions:
     * height of centre of rotation above the ground, metres
     * degrees anticlockwise for Port-side usecase, 0 = coral at top 
     */
    public static class Presets
    {
      public static final ArmPos startPosition          = new ArmPos(0.616,  0);
      public static final ArmPos climbSafePosition      = new ArmPos(DiffectorGeometry.safeElevation,  90); // TODO
      public static final ArmPos climbPosition          = new ArmPos(0.425, 90);
      public static final ArmPos netPosition            = new ArmPos(DiffectorGeometry.maxZ, 150, false);
      public static final ArmPos algae3Position         = new ArmPos(1.08, 113);
      public static final ArmPos algae2Position         = new ArmPos(0.65, 113);
      public static final ArmPos processorPosition      = new ArmPos(0.43,  90);
      public static final ArmPos coral4Position         = new ArmPos(1.72, 325);
      public static final ArmPos coral3Position         = new ArmPos(1.05, 325);
      public static final ArmPos coral2Position         = new ArmPos(0.675, 325);
      public static final ArmPos coral1ClawPosition     = new ArmPos(DiffectorGeometry.algaeClawElevation,  76);
      public static final ArmPos coral1Position         = new ArmPos(DiffectorGeometry.coralFunnelElevation, 210);
      public static final ArmPos coralIntakePosition    = new ArmPos(1.16, 215); // TODO
      public static final ArmPos coralIntakeAltPosition = new ArmPos(1.09, 215); // TODO
      public static final ArmPos coralClawPosition      = new ArmPos(0.73, 129); // TODO
      public static final ArmPos coralStowPosition      = new ArmPos(0.72,   0);
      public static final ArmPos algaeIntakePosition    = new ArmPos(0.47,  70); // +20cm for testing
      public static final ArmPos algaeStowPosition      = new ArmPos(0.80, 180);

      public static final ArrayList<ArmPos> lowDiffectorPositions = new ArrayList<ArmPos>()
      {{
        add(startPosition);
        add(algaeIntakePosition);
        add(climbPosition);
        add(algaeStowPosition);
      }};

      public static final ArrayList<ArmPos> highDiffectorPositions = new ArrayList<ArmPos>()
      {{
        add(netPosition);
        add(coral4Position);
        add(coral3Position);
      }};
    }

    /** Raw value when fully released, indicating string has snapped or the sensor is unavailable */
    public static final double potErrValue = 0.06;

    public static final InterpolatingDoubleTreeMap potInterpolation = new InterpolatingDoubleTreeMap()
    {
      {
        put(0.09, 0.36);
        put(0.15, 0.47);
        put(0.25, 0.61);
        put(0.30, 0.70);
        put(0.43, 0.90);
        put(0.52, 1.02);
        put(0.57, 1.10);
        put(0.64, 1.20);
        put(0.71, 1.31);
        put(0.72, 1.33);
        put(0.86, 1.54);
        put(0.91, 1.61);
        put(0.94, 1.66);
        put(0.96, 1.69);
        put(0.98, 1.72);
        put(0.99, 1.74);
        put(1.00, 1.76);
      }
    };
  }

  public static final class Manipulators 
  {
    /* Coral manipulator speeds */
    public static final double coralLvl4DeliverySpeed = 0.25;
    public static final double coralDeliverySpeed     = 0.30;
    public static final double coralHoldingSpeed      = -0.10;

    /* Algae manipulator speeds */
    public static final double algaeIntakeSpeed    = -1;
    public static final double algaeHoldingVoltage = -12;
    public static final double algaeNetSpeed       =  1;
    public static final double algaeProcessorSpeed =  0.9;
    public static final double algaeHeldCurrent    = 40;
    //public static final double algaeReleaseCurrent =  4;

    /** Algae net shooting range for rotation snapping, m */
    public static final double algaeRange = 2.5;
  }

  public static final class ClimberConstants
  {
    public static final double stowWinchPos    = 0.0;
    public static final double safeWinchPos    = 4.5;
    public static final double startDrivePos   = 4.7;
    public static final double offGroundPos    = 2.5;
    public static final double prepareWinchPos = 5.5; // TODO
    public static final double climbWinchPos   = 2.0; // TODO
    /** The furthest into the robot the climber can attempt to go whilst balancing */
    public static final double climbActiveInnerLimit = 1.7; // TODO
    /** The furthest out of the robot the climber can attempt to go whilst balancing */
    public static final double climbActiveOuterLimit = 2.3; // TODO
    /** Ideal robot pitch when hanging, in degrees */
    public static final double targetRobotClimbPitch = 3; 
  }

  public static final class LEDStrip
  {
    /**
     * Constants used by the addressable LED classes.
     */
    /** PWM port the strip is connected to. */
    public static final int LEDPWMPort = IDConstants.LEDPWM; 
    /** # of LED's in the strip, if more than one strip daisy-chained, total # of LED's */
    public static final int lightsLen = 118; 
    /** default width for a partial display layer. a good number is about 1/4 lightsLen */
    public static final int viewWidth = 30;
    /** Start and end positions for Volaans displays */
    public static final int stbdStatusStart = 0; //86;
    public static final int stbdStatusWidth = 30; //30;
    public static final int portStatusStart = 83; //0;
    public static final int portStatusWidth = 30; //30;
    public static final int stbdHaloStart = 30; //58;
    public static final int stbdHaloWidth = 26; //28;
    public static final int portHaloStart = 56; //30;
    public static final int portHaloWidth = 27; //28;
    /** LED # at 0 degrees */
    public static final int startOffset = 1; 
    /** used to calculate the LED pointing in a particular direction */
    public static final double degreesPerLED = 360/lightsLen;  
    /** default background / off Color */
    public static final Color defaultBackColor = Color.kRed; 
    /** default foreground / on Color */
    public static final Color defaultFrontColor = Color.kGreen; 
    /** default Color for border dots */
    public static final Color displayBorderColor = Color.kBlue; 
    /** default # of segments in a status display */
    public static final int defaultStatusSegments = 3; 
    /** length of pointer layer above which a gradient will be applied rather than solid colour */
    public static final int pointerGradientThreshold = 5; 
    /** start and end LED #'s for the 'starboard' segment */
    public static final int stbdLEDsStart = 0; 
    public static final int stbdLEDsEnd = 57;
    /** start and end LED #'s for the 'port' segment */
    public static final int portLEDsStart = 58; 
    public static final int portLEDsEnd = 115;
    /** maximum colour layers in disco mode */
    public static final int discoMax = 10;  
    /** minimum colour layers in disco mode */
    public static final int discoMin = 3;  
    /**disco layers will randomly die of age between discoAgeLimit and 2x discoAgeLimit seconds */
    public static final int discoAgeLimit = 10; 
    /** the probability of accel or growthrate changing in any update is 1 - this: (1 - 0.9 = 0.1 = 10% chance of change) */
    public static final double discoChangeChance = 0.9; 
    /** used to calculate maxLen based on viewWidth. */
    public static final double discoMaxLenMultiplier = 0.3;
    /** used to calculate maxVel based on viewWidth, at maximum velocity it will take 1/this seconds to traverse the strip. */
    public static final double discoMaxVelMultiplier = 0.2; 
    /** used to calculate maxAccel, maxVel will be multiplied by this to get the value. */
    public static final double discoMaxAccelMultiplier = 0.02; 
    /** used to calculate maxGrow based on viewWidth. */
    public static final double discoMaxGrowMultiplier = 0.05; 
    /** multipied by maxGrow to get maxGrowRate */
    public static final double discoMaxGrowRateMultiplier = 0.01; 
    /** at least one color value (r,g,b) must be above this for the colour to be valid */
    public static final double discoColorThreshold = 0.5; 
  }
}
