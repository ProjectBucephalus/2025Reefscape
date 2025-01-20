package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public static TalonFXConfiguration diffectorFXConfig = new TalonFXConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

        /* Diffector Motor Config (Empty) */
        diffectorFXConfig.Slot0.kS = Constants.Diffector.diffectorMotorKPEmpty;
        diffectorFXConfig.Slot0.kV = Constants.Diffector.diffectorMotorKIEmpty;
        diffectorFXConfig.Slot0.kA = Constants.Diffector.diffectorMotorKDEmpty;
        diffectorFXConfig.Slot0.kP = Constants.Diffector.diffectorMotorKPEmpty;
        diffectorFXConfig.Slot0.kI = Constants.Diffector.diffectorMotorKIEmpty;
        diffectorFXConfig.Slot0.kD = Constants.Diffector.diffectorMotorKDEmpty;
        
        /* Diffector Motor Config (Algae or Coral) */
        diffectorFXConfig.Slot1.kS = Constants.Diffector.diffectorMotorKPOneItem;
        diffectorFXConfig.Slot1.kV = Constants.Diffector.diffectorMotorKIOneItem;
        diffectorFXConfig.Slot1.kA = Constants.Diffector.diffectorMotorKDOneItem;
        diffectorFXConfig.Slot1.kP = Constants.Diffector.diffectorMotorKPOneItem;
        diffectorFXConfig.Slot1.kI = Constants.Diffector.diffectorMotorKIOneItem;
        diffectorFXConfig.Slot1.kD = Constants.Diffector.diffectorMotorKDOneItem;        

        /* Diffector Motor Config (Algae and Coral) */
        diffectorFXConfig.Slot2.kS = Constants.Diffector.diffectorMotorKPTwoItem;
        diffectorFXConfig.Slot2.kV = Constants.Diffector.diffectorMotorKITwoItem;
        diffectorFXConfig.Slot2.kA = Constants.Diffector.diffectorMotorKDTwoItem;
        diffectorFXConfig.Slot2.kP = Constants.Diffector.diffectorMotorKPTwoItem;
        diffectorFXConfig.Slot2.kI = Constants.Diffector.diffectorMotorKITwoItem;
        diffectorFXConfig.Slot2.kD = Constants.Diffector.diffectorMotorKDTwoItem;

        /* Diffector MotionMagic Config */
        diffectorFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Diffector.diffectorMotionMagicCruise;
        diffectorFXConfig.MotionMagic.MotionMagicAcceleration = Constants.Diffector.diffectorMotionMagicAccel;

    }
}