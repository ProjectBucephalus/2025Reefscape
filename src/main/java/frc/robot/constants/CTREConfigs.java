package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class CTREConfigs 
{
  public static final TalonFXConfiguration climberWinchFXConfig = new TalonFXConfiguration();
  public static final TalonFXConfiguration diffectorFXConfig = new TalonFXConfiguration();

  public CTREConfigs()
  {
    /* Diffector Motor Gneral Config */
    diffectorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    diffectorFXConfig.Feedback.SensorToMechanismRatio = Constants.DiffectorConstants.gearboxRatio;

    /* Diffector Motor Config (Default) */
    diffectorFXConfig.Slot0.kG = Constants.DiffectorConstants.diffectorMotorKG;
    diffectorFXConfig.Slot0.kS = Constants.DiffectorConstants.diffectorMotorKS;
    diffectorFXConfig.Slot0.kV = Constants.DiffectorConstants.diffectorMotorKV;
    diffectorFXConfig.Slot0.kP = Constants.DiffectorConstants.diffectorMotorKP;
    diffectorFXConfig.Slot0.kI = Constants.DiffectorConstants.diffectorMotorKI;
    diffectorFXConfig.Slot0.kD = Constants.DiffectorConstants.diffectorMotorKD;
    
    /* Diffector Motor Config (Virtual Spring) */
    diffectorFXConfig.Slot1.kG = Constants.DiffectorConstants.diffectorMotorKGSpring;
    diffectorFXConfig.Slot1.kS = Constants.DiffectorConstants.diffectorMotorKSSpring;
    diffectorFXConfig.Slot1.kV = Constants.DiffectorConstants.diffectorMotorKVSpring;
    diffectorFXConfig.Slot1.kP = Constants.DiffectorConstants.diffectorMotorKPSpring;
    diffectorFXConfig.Slot1.kI = Constants.DiffectorConstants.diffectorMotorKISpring;
    diffectorFXConfig.Slot1.kD = Constants.DiffectorConstants.diffectorMotorKDSpring;

    /* Diffector MotionMagic Default Config */
    diffectorFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.DiffectorConstants.diffectorCruise;
    diffectorFXConfig.MotionMagic.MotionMagicAcceleration = Constants.DiffectorConstants.diffectorRotationAcceleration;

    /* Climber Values */
    climberWinchFXConfig.Feedback.SensorToMechanismRatio = Constants.ClimberConstants.winchGearRatio;
    climberWinchFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimberConstants.winchDefaultCruise;
    climberWinchFXConfig.MotionMagic.MotionMagicAcceleration = Constants.ClimberConstants.winchMotionMagicAccel;
    climberWinchFXConfig.Slot0.kP = Constants.ClimberConstants.winchKP;
    climberWinchFXConfig.Slot0.kI = Constants.ClimberConstants.winchKI;
    climberWinchFXConfig.Slot0.kD = Constants.ClimberConstants.winchKD;
    climberWinchFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberWinchFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }
}