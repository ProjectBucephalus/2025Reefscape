package frc.robot.constants;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Test;
import frc.robot.subsystems.Diffector.CargoStates;

public final class Constants 
{
    public static final class Rumbler 
    {
        public static final double driver_Default = 1;
        public static final double coDriver_Default = 1;  
    }

    public static final class Control
    {
        public static final double stickDeadband = 0.1;
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
        public static final int[] validIDs = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
        public static final String limeLightName = "limelight";
    }

    public static final class Swerve
    {
        public static final double initialHeading = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 1.8; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        /* Rotation Control PID Values */
        public static final double rotationKP = 3; //TODO: Tune to robot
        public static final double rotationKI = 0;
        public static final double rotationKD = 0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5; // ERROR: Unsure if this value works as it should
        /** Radians per Second */
        public static final double maxAngularVelocity = 6;
    }

    public static final class Auto
    {   
        /** m/s */
        public static final double pathplannerMaxSpeed = 5;
        /** m/s^2 */
        public static final double pathplannerMaxAcceleration = 5.5;
        /** degrees/s */
        public static final double pathplannerMaxAngularSpeed = 720;
        /** degrees/s^2 */
        public static final double pathplannerMaxAngularAcceleration = 1050;
        public static final PathConstraints defaultConstraints = new PathConstraints
            (pathplannerMaxSpeed, pathplannerMaxAcceleration, pathplannerMaxAngularSpeed, pathplannerMaxAngularAcceleration);
        
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
                put("ra" , new AutoMapping("test", null));
                put("rb" , new AutoMapping("test", null));
                put("rc" , new AutoMapping("test", null));
                put("rd" , new AutoMapping("test", null));
                put("re" , new AutoMapping("test", null));
                put("rf" , new AutoMapping("test", null));
                put("rg" , new AutoMapping("test", null));
                put("rh" , new AutoMapping("test", null));
                put("ri" , new AutoMapping("test", null));
                put("rj" , new AutoMapping("test", null));
                put("rk" , new AutoMapping("test", null));
                put("rl" , new AutoMapping("test", null));
                put("cl1", new AutoMapping("test", null));
                put("cl2", new AutoMapping("test", null));
                put("cl3", new AutoMapping("test", null));
                put("cl4", new AutoMapping("test", null));
                put("cl5", new AutoMapping("test", null));
                put("cl6", new AutoMapping("test", null));
                put("cl7", new AutoMapping("test", null));
                put("cl8", new AutoMapping("test", null));
                put("cl9", new AutoMapping("test", null));
                put("cr1", new AutoMapping("test", null));
                put("cr2", new AutoMapping("test", null));
                put("cr3", new AutoMapping("test", null));
                put("cr4", new AutoMapping("test", null));
                put("cr5", new AutoMapping("StationSouth", () -> new Test("Pickup", "picked up cR5")));
                put("cr6", new AutoMapping("test", null));
                put("cr7", new AutoMapping("test", null));
                put("cr8", new AutoMapping("test", null));
                put("cr9", new AutoMapping("test", null));
                put("a1" , new AutoMapping("test", null));
                put("a2" , new AutoMapping("test", null));                
                put("a3" , new AutoMapping("test", null));                
                put("a4" , new AutoMapping("test", null));                
                put("a5" , new AutoMapping("test", null));                
                put("a6" , new AutoMapping("test", null));
                put("sc1", new AutoMapping("test", null));
                put("sc2", new AutoMapping("test", null));
                put("sc3", new AutoMapping("test", null));
                put("sa1", new AutoMapping("test", null));
                put("sa2", new AutoMapping("test", null));
                put("sa3", new AutoMapping("test", null));
                put("b1" , new AutoMapping("test", null));
                put("b2" , new AutoMapping("test", null));
                put("b3" , new AutoMapping("test", null));
                put("p"  , new AutoMapping("test", null));                
            }
        };

        public static final String defaultAuto = "t5,cR5,w3.5,cR5";
    }

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

        public static final double maxRotation = 3;
        public static final double maxAbsPos = maxRotation * 360;
        public static final double turnBackThreshold = 135;
        public static final double returnPos = 0;
        public static final double stowThreshold = 1;
    }

    public static final class GamePiecesManipulator 
    {
        public static final int coralMotorID = IDConstants.coralMotorID;
        public static final int algaeMotorID = IDConstants.algaeMotorID;

        public static final int coralBeamBreak1DigitalInput = 0;
        public static final int coralBeamBreak2DigitalInput = 1;
        public static final int algaeBeamBreakDigitalInput = 2;

        /* Coral manipulator speeds */
        public static final double coralManipulatorIntakeSpeed = 0;
        public static final double coralManipulatorDeliverySpeed = 0.9;
        public static final double coralManipulatorHoldingSpeed = 0.2;

        /* Algae manipulator speeds */
        public static final double algaeManipulatorIntakeSpeed = -0.7;
        public static final double algaeManipulatorHoldingVoltage = 0.1;
        public static final double algaeManipulatorNetSpeed = 1;
        public static final double algaeManipulatorProcessorSpeed = 0.6;
        public static final double algaeManipulatorEmptySpeed = 0;
    }

    public static final class Intake // TODO: Speeds and Angles must be tuned to the specific robot
    {
        public static final int topIntakeID = IDConstants.mTopIntakeID;
        public static final int bottomIntakeID = IDConstants.mBottomIntakeID;
        public static final int topArmID = IDConstants.mTopArmID;
        public static final int bottomArmID = IDConstants.mBottomArmID;

        /* Intake motors speeds */
        public static final double coralIntakeMotorSpeed = 0.8;
        public static final double algaeIntakeMotorSpeed = 0.8;
        public static final double coralTransferMotorSpeed = 0;
        public static final double algaeTransferMotorSpeed = 0;
        public static final double climbingIntakeMotorSpeed = 0;
        public static final double standByMotorSpeed = 0;
        public static final double stowedMotorSpeed = 0;

        /* Top intake arm positions 
         * TODO: Put in Degrees for the arm top and bottom position in this comment
         */
        public static final double topCoralIntakeArmTarget = 0;
        public static final double topAlgaeIntakeArmTarget = 0;
        public static final double topClimbingArmTarget = 0;
        public static final double topStandByArmTarget = 0;
        public static final double topStowedArmTarget = 0;
        public static final double topCoralTransferArmTarget = 0;
        public static final double topAlgaeTransferArmTarget = 0;

        /* Bottom intake arm positions */
        public static final double bottomCoralIntakeArmTarget = 0;
        public static final double bottomAlgaeIntakeArmTarget = 0;
        public static final double bottomClimbingArmTarget = 0;
        public static final double bottomStandByArmTarget = 0;
        public static final double bottomStowedArmTarget = 0;
        public static final double bottomCoralTransferArmTarget = 0;
        public static final double bottomAlgaeTransferArmTarget = 0;

        /* Top arm PID + FeedForward values */
        public static final double topArmSpringKP = 1;
        public static final double topArmSpringKI = 0;
        public static final double topArmSpringKD = 0;
        public static final double topArmStopKP = 12.5;
        public static final double topArmStopKI = 0;
        public static final double topArmStopKD = 0;
        
        public static final double topArmKS = 0.15;
        public static final double topArmKG = 0.15;

        /* Bottom arm PID values */
        public static final double bottomArmKP = 1;
        public static final double bottomArmKI = 0;
        public static final double bottomArmKD = 0.25;

        /* Arm MotionMagic values */
        public static final double targetVelocity = 0.25;
        public static final double targetAcceleration = 0.25;

        /* Arm ratios */
        public static final double topArmRatio = 16.7;
        public static final double bottomArmRatio = 1;
    }

    public static final class Climber
    {
        public static final int clawID = IDConstants.climberClawMotorID;
        public static final int winchID = IDConstants.climberWinchMotorID;

        public static final double initSpeed = 0;
        public static final double deploySpeed = 0.8;
        public static final double climbSpeed = 0.8;

        public static final double initClawPos = 0;
        public static final double initWinchPos = 0;
        public static final double deployClawPos = 90;
        public static final double deployWinchPos = 0;
        public static final double climbClawPos = 0;
        public static final double climbWinchPos = 0;
    }
}
