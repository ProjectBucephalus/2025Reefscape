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
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.angleCurrentLimit;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.SwerveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.SwerveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.SwerveConstants.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.driveCurrentLimit;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.SwerveConstants.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;

        /* Diffector Motor Config */
        diffectorFXConfig.Slot0.kP = Constants.DiffectorConstants.diffectorMotorKP;
        diffectorFXConfig.Slot0.kI = Constants.DiffectorConstants.diffectorMotorKI;
        diffectorFXConfig.Slot0.kD = Constants.DiffectorConstants.diffectorMotorKD;
    }
}