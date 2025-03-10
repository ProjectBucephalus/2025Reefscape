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

    /* Diffector Motor Config (Empty) */
    diffectorFXConfig.Slot0.kG = Constants.DiffectorConstants.diffectorMotorKGEmpty;
    diffectorFXConfig.Slot0.kS = Constants.DiffectorConstants.diffectorMotorKSEmpty;
    diffectorFXConfig.Slot0.kV = Constants.DiffectorConstants.diffectorMotorKVEmpty;
    diffectorFXConfig.Slot0.kP = Constants.DiffectorConstants.diffectorMotorKPEmpty;
    diffectorFXConfig.Slot0.kI = Constants.DiffectorConstants.diffectorMotorKIEmpty;
    diffectorFXConfig.Slot0.kD = Constants.DiffectorConstants.diffectorMotorKDEmpty;
    
    /* Diffector Motor Config (Algae or Coral) */
    diffectorFXConfig.Slot1.kG = Constants.DiffectorConstants.diffectorMotorKGOneItem;
    diffectorFXConfig.Slot1.kS = Constants.DiffectorConstants.diffectorMotorKSOneItem;
    diffectorFXConfig.Slot1.kV = Constants.DiffectorConstants.diffectorMotorKVOneItem;
    diffectorFXConfig.Slot1.kP = Constants.DiffectorConstants.diffectorMotorKPOneItem;
    diffectorFXConfig.Slot1.kI = Constants.DiffectorConstants.diffectorMotorKIOneItem;
    diffectorFXConfig.Slot1.kD = Constants.DiffectorConstants.diffectorMotorKDOneItem;        

    /* Diffector Motor Config (Algae and Coral) */
    diffectorFXConfig.Slot2.kG = Constants.DiffectorConstants.diffectorMotorKGTwoItem;
    diffectorFXConfig.Slot2.kS = Constants.DiffectorConstants.diffectorMotorKSTwoItem;
    diffectorFXConfig.Slot2.kV = Constants.DiffectorConstants.diffectorMotorKVTwoItem;
    diffectorFXConfig.Slot2.kP = Constants.DiffectorConstants.diffectorMotorKPTwoItem;
    diffectorFXConfig.Slot2.kI = Constants.DiffectorConstants.diffectorMotorKITwoItem;
    diffectorFXConfig.Slot2.kD = Constants.DiffectorConstants.diffectorMotorKDTwoItem;

    /* Diffector MotionMagic Config */
    diffectorFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.DiffectorConstants.diffectorMotionMagicCruise / Constants.DiffectorConstants.gearboxRatio;
    diffectorFXConfig.MotionMagic.MotionMagicAcceleration = Constants.DiffectorConstants.diffectorMotionMagicAccel / Constants.DiffectorConstants.gearboxRatio;

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