package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.COTSTalonFXSwerveConstants;
import frc.robot.util.SwerveModuleConstants;

import frc.robot.util.GeoFenceObject;
import frc.robot.util.GeoFenceObject.ObjectTypes;

public final class Constants 
{
    
    public static final class ControlConstants
    {
        public static final double stickDeadband = 0.1;
        /** Normal maximum robot speed, relative to maximum uncapped speed */
        public static final double maxThrottle = 0.3;
        /** Minimum robot speed when braking, relative to maximum uncapped speed */
        public static final double minThrottle = 0.1;
        /** Normal maximum rotational robot speed, relative to maximum uncapped rotational speed */
        public static final double maxRotThrottle = 1;
        /** Minimum rotational robot speed when braking, relative to maximum uncapped rotational speed */
        public static final double minRotThrottle = 0.5;
        /** Scales manual rotation speed */
        public static final double manualRotationScalar = 15;
        /** Maximum robot rotation speed */
        public static final double maxRotationSpeed = 3;
        /** Steering overswing compensation factor */
        public static final double overswingReduction = 2;
    }
    
    public final class GeoFencing // TODO: Move this to FieldConstants file.
    {   
        // Relative to the centre of the robot, in direction the robot is facing
        // These values are the distance in metres to the virtual wall the robot will stop at
        // 0 means the wall is running through the middle of the robot
        // negative distances will have the robot start outside the area, and can only move into it
        /** Metres the robot can travel left */
        public static final double fieldLeft = 3;

        /** Metres the robot can travel right */
        public static final double fieldRight = 6;

        /** Metres the robot can travel forwards */
        public static final double fieldFront = 3;

        /** Metres the robot can travel back */
        public static final double fieldBack = 3;    

        public static final GeoFenceObject field = new GeoFenceObject
        (
            -fieldBack, 
            -fieldRight, 
            fieldFront, 
            fieldLeft, 
            0.5,
            0.0,
            ObjectTypes.walls
        );
        
        public static final GeoFenceObject test1 = new GeoFenceObject(1.25, 1.25, 0.2);
        public static final GeoFenceObject test2 = new GeoFenceObject(-0.5,-2.5,0.5,-4,0.5,0,ObjectTypes.box);
        public static final GeoFenceObject test3 = new GeoFenceObject(0, 0.75, -1.5, 1.8, 0.5, 0.2);

        public static final GeoFenceObject[] fieldGeoFence = {field,test1,test2,test3};
        
        /** Radius from robot centre in metres where geofence is triggered */
        public static final double robotRadius = 0.55;
        
        //public static OdometryReadout fieldReadout = new OdometryReadout(0.5);
        //fieldReadout.addRectangle(fieldGeoFence[0].getObject);
    }

    public static final class Swerve
    {
        public static final int pigeonID = 5;

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
        //public static final double angleGearRatioAlt = 12.17; // ERROR: The physical gearing on the specific robot is built wrong, remove this if all swerve modules are built correctly! Should be ~13.37

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

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32 / 12; 
        public static final double driveKV = 1.51 / 12;
        public static final double driveKA = 0.27 / 12;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 1; // ERROR: Unsure if this value works as it should
        /** Radians per Second */
        public static final double maxAngularVelocity = 3;
        
        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        // # 5985 Fix: With the additiona of persistant calibration, the angleOffset values should not need to be changed # //
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0
        { 
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1
        { 
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2
        { 
            public static final int driveMotorID = 16;
            public static final int angleMotorID = 18;
            public static final int canCoderID = 17;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3
        { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(270.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants // TODO: The below constants are used in the example auto, and must be tuned to specific robot
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
}
