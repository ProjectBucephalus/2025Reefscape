package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Diffector.CargoStates;
import frc.robot.util.COTSTalonFXSwerveConstants;
import frc.robot.util.SwerveModuleConstants;

public final class Constants 
{
    public static final class Diffector
    {
        public static final int ucMotorID = IDConstants.ucMotorID;
        public static final int uaMotorID = IDConstants.uaMotorID;

        public static final double diffectorMotorKSEmpty = 0;
        public static final double diffectorMotorKVEmpty = 0;
        public static final double diffectorMotorKAEmpty = 0;
        public static final double diffectorMotorKPEmpty = 0;
        public static final double diffectorMotorKIEmpty = 0;
        public static final double diffectorMotorKDEmpty = 0;

        public static final double diffectorMotorKSOneItem = 0;
        public static final double diffectorMotorKVOneItem = 0;
        public static final double diffectorMotorKAOneItem = 0;
        public static final double diffectorMotorKPOneItem = 0;
        public static final double diffectorMotorKIOneItem = 0;
        public static final double diffectorMotorKDOneItem = 0;

        public static final double diffectorMotorKSTwoItem = 0;
        public static final double diffectorMotorKVTwoItem = 0;
        public static final double diffectorMotorKATwoItem = 0;
        public static final double diffectorMotorKPTwoItem = 0;
        public static final double diffectorMotorKITwoItem = 0;
        public static final double diffectorMotorKDTwoItem = 0;

        public static final double diffectorMotionMagicCruise = 0;
        public static final double diffectorMotionMagicAccel = 0;

        /** Output sprocket rotations per motor rotation */
        public static final double gearboxRatio = 8/60;
        /** Pitch Diameter of the sprocket, in mm */
        public static final double sprocketPitchDiameter = 36.576;
        /** Ratio of output sprocket to arm sprocket (output sprocket teeth/arm sprocket teeth) */
        public static final double sprocketRatio = 18/72;

        /** 
         * Number of chain mm moved for one motor degree. 
         * Sprocket rotations per motor rotation * mm of chain moved per sprocket rotation (pitch diam. * pi), 
         * all divided by 360 to convert rotations to degrees 
         */
        public static final double travelRatio = ((gearboxRatio * sprocketPitchDiameter * Math.PI) / 360) / 2;
        /** 
         * Number of arm degrees moved for one motor degree of a single motor 
         * Output sprocket rotations per motor rotation * output sprocket to arm sprocket ratio,
         * halved to account for the dynamics of a sprocket with only one moving chain
         */
        public static final double rotationRatio = gearboxRatio * sprocketRatio / 2;

        public static final CargoStates startingCargoState = CargoStates.empty;
    }

    public static final class Rumbler 
    {
        public static final double driver_Default = 1;
        public static final double coDriver_Default = 1;  
    }

    public static final class Control
    {
        public static final double stickDeadband = 0.1;
        /** Normal maximum robot speed, relative to maximum uncapped speed */
        public static final double maxThrottle = 0.7;
        /** Minimum robot speed when braking, relative to maximum uncapped speed */
        public static final double minThrottle = 0.1;
        /** Normal maximum rotational robot speed, relative to maximum uncapped rotational speed */
        public static final double maxRotThrottle = 1;
        /** Minimum rotational robot speed when braking, relative to maximum uncapped rotational speed */
        public static final double minRotThrottle = 0.5;
        /** Scales manual rotation speed */
        public static final double manualRotationScalar = 15;
        /** Maximum robot rotation speed */
        public static final double maxRotationSpeed = 2;
        /** Steering overswing compensation factor */
        public static final double overswingReduction = 2;
    }

    public static final class Vision
    {
        public static final int[] validIDs = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
        public static final String limeLightName = "limelight";
    }

    public static final class Swerve
    {
        public static final int pigeonID = IDConstants.pigeonID;
        public static final double initialHeading = 0;

        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);
        
        /* Drivetrain Constants */
        public static final double trackWidth = 0.48;
        public static final double wheelBase = 0.48;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;
        /* public static final double angleGearRatioAlt = 12.17; 
        * ERROR: The physical gearing on the specific robot is built wrong, remove this if all swerve modules are built correctly! 
        * Should be ~13.37 */

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 35;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 1.8; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Rotation Control PID Values */
        public static final double rotationKP = 0.06; //TODO: Tune to robot
        public static final double rotationKI = 0;
        public static final double rotationKD = 0.00075;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32 / 12; 
        public static final double driveKV = 1.51 / 12;
        public static final double driveKA = 0.27 / 12;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5; // ERROR: Unsure if this value works as it should
        /** Radians per Second */
        public static final double maxAngularVelocity = 3;
        
        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        // # 5985 Fix: With the additional of persistant calibration, the angleOffset values should not need to be changed # //
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0
        { 
            public static final int driveMotorID = IDConstants.mod0DriveMotorID;
            public static final int angleMotorID = IDConstants.mod0AngleMotorID;
            public static final int canCoderID = IDConstants.mod0CanCoderID;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1
        { 
            public static final int driveMotorID = IDConstants.mod1DriveMotorID;
            public static final int angleMotorID = IDConstants.mod1AngleMotorID;
            public static final int canCoderID = IDConstants.mod1CanCoderID;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2
        { 
            public static final int driveMotorID = IDConstants.mod2DriveMotorID;
            public static final int angleMotorID = IDConstants.mod2AngleMotorID;
            public static final int canCoderID = IDConstants.mod2CanCoderID;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3
        { 
            public static final int driveMotorID = IDConstants.mod3DriveMotorID;
            public static final int angleMotorID = IDConstants.mod3AngleMotorID;
            public static final int canCoderID = IDConstants.mod3CanCoderID;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(270.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Auto // TODO: The below constants are used in the example auto, and must be tuned to specific robot
    {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Intake // TODO: Speeds and Angles must be tuned to the specific robot
    {
        public static final class MotorID
        {
            public static final int mTopIntakeID = IDConstants.mTopIntakeID;
            public static final int mBottomIntakeID = IDConstants.mBottomIntakeID;
            public static final int mTopArmID = IDConstants.mTopArmID;
            public static final int mBottomArmID = IDConstants.mBottomArmID;
        }

        public static final class MotorSpeeds
        {
            /* Constants for the intake motors speeds */
            public static final double mCoralIntakeMotorSpeed = 0.8;
            public static final double mAlgaeIntakeMotorSpeed = 0.8;
            public static final double mCoralTransferMotorSpeed = 0;
            public static final double mAlgaeTransferMotorSpeed = 0;
            public static final double mClimbingIntakeMotorSpeed = 0;
            public static final double mStandByMotorSpeed = 0;
            public static final double mStowedMotorSpeed = 0;
        }

        public static final class ArmPosition
        {
            /* Top intake arm positions 
             * TODO: Put in Degrees for the arm top and bottom position in this comment
            */
            public static final double mTopCoralIntakeArmTarget = 0;
            public static final double mTopAlgaeIntakeArmTarget = 0;
            public static final double mTopClimbingArmTarget = 0;
            public static final double mTopStandByArmTarget = 0;
            public static final double mTopStowedArmTarget = 0;
            public static final double mTopCoralTransferArmTarget = 0;
            public static final double mTopAlgaeTransferArmTarget = 0;

            /* Bottom intake arm positions */
            public static final double mBottomCoralIntakeArmTarget = 0;
            public static final double mBottomAlgaeIntakeArmTarget = 0;
            public static final double mBottomClimbingArmTarget = 0;
            public static final double mBottomStandByArmTarget = 0;
            public static final double mBottomStowedArmTarget = 0;
            public static final double mBottomCoralTransferArmTarget = 0;
            public static final double mBottomAlgaeTransferArmTarget = 0;
        }
    }

    public static final class Climber
    {
        public static final class MotorID
        {
            public static final int m_Claws = IDConstants.climberClawMotorID;
            public static final int m_Winch = IDConstants.climberWinchMotorID;
        }

        public static final class MotorSpeeds
        {
            public static final double m_InitSpeed = 0;
            public static final double m_DeploySpeed = 0.8;
            public static final double m_ClimbSpeed = 0.8;
        }

        public static final class ClimberPos 
        {
            public static final double m_InitClawPos = 0;
            public static final double m_InitWinchPos = 0;
            public static final double m_DeployClawPos = 90;
            public static final double m_DeployWinchPos = 0;
            public static final double m_ClimbClawPos = 0;
            public static final double m_ClimbWinchPos = 0;
        }
    }
}
