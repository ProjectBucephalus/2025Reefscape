package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class CTREConfigs 
{
  public static final TalonFXConfiguration climberWinchFXConfiguration = new TalonFXConfiguration();
  public static final TalonFXConfiguration intakeTopArmFXConfig = new TalonFXConfiguration();
  public static final TalonFXConfiguration intakeBottomArmFXConfig = new TalonFXConfiguration();
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
    diffectorFXConfig.Slot0.kA = Constants.DiffectorConstants.diffectorMotorKAEmpty;
    diffectorFXConfig.Slot0.kP = Constants.DiffectorConstants.diffectorMotorKPEmpty;
    diffectorFXConfig.Slot0.kI = Constants.DiffectorConstants.diffectorMotorKIEmpty;
    diffectorFXConfig.Slot0.kD = Constants.DiffectorConstants.diffectorMotorKDEmpty;
    
    /* Diffector Motor Config (Algae or Coral) */
    diffectorFXConfig.Slot1.kG = Constants.DiffectorConstants.diffectorMotorKGOneItem;
    diffectorFXConfig.Slot1.kS = Constants.DiffectorConstants.diffectorMotorKSOneItem;
    diffectorFXConfig.Slot1.kV = Constants.DiffectorConstants.diffectorMotorKVOneItem;
    diffectorFXConfig.Slot1.kA = Constants.DiffectorConstants.diffectorMotorKAOneItem;
    diffectorFXConfig.Slot1.kP = Constants.DiffectorConstants.diffectorMotorKPOneItem;
    diffectorFXConfig.Slot1.kI = Constants.DiffectorConstants.diffectorMotorKIOneItem;
    diffectorFXConfig.Slot1.kD = Constants.DiffectorConstants.diffectorMotorKDOneItem;        

    /* Diffector Motor Config (Algae and Coral) */
    diffectorFXConfig.Slot2.kG = Constants.DiffectorConstants.diffectorMotorKGTwoItem;
    diffectorFXConfig.Slot2.kS = Constants.DiffectorConstants.diffectorMotorKSTwoItem;
    diffectorFXConfig.Slot2.kV = Constants.DiffectorConstants.diffectorMotorKVTwoItem;
    diffectorFXConfig.Slot2.kA = Constants.DiffectorConstants.diffectorMotorKATwoItem;
    diffectorFXConfig.Slot2.kP = Constants.DiffectorConstants.diffectorMotorKPTwoItem;
    diffectorFXConfig.Slot2.kI = Constants.DiffectorConstants.diffectorMotorKITwoItem;
    diffectorFXConfig.Slot2.kD = Constants.DiffectorConstants.diffectorMotorKDTwoItem;

    /* Diffector MotionMagic Config */
    diffectorFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.DiffectorConstants.diffectorMotionMagicCruise;
    diffectorFXConfig.MotionMagic.MotionMagicAcceleration = Constants.DiffectorConstants.diffectorMotionMagicAccel;

    /* Intake Top Arm PID Config, Spring Behaviour */
    intakeTopArmFXConfig.Slot0.kP = Constants.IntakeConstants.topArmSpringKP;
    intakeTopArmFXConfig.Slot0.kI = Constants.IntakeConstants.topArmSpringKI;
    intakeTopArmFXConfig.Slot0.kD = Constants.IntakeConstants.topArmSpringKD;
    intakeTopArmFXConfig.Slot0.kS = Constants.IntakeConstants.topArmKS;
    intakeTopArmFXConfig.Slot0.kG = -Constants.IntakeConstants.topArmKG;

    /* Intake Top Arm PID Config, Stop Behaviour */
    intakeTopArmFXConfig.Slot1.kP = Constants.IntakeConstants.topArmStopKP;
    intakeTopArmFXConfig.Slot1.kI = Constants.IntakeConstants.topArmStopKI;
    intakeTopArmFXConfig.Slot1.kD = Constants.IntakeConstants.topArmStopKD;
    intakeTopArmFXConfig.Slot1.kS = Constants.IntakeConstants.topArmKS;
    intakeTopArmFXConfig.Slot1.kG = Constants.IntakeConstants.topArmKG;

    /* Intake Bottom Arm PID Config */
    intakeBottomArmFXConfig.Slot0.kP = Constants.IntakeConstants.bottomArmKP;
    intakeBottomArmFXConfig.Slot0.kI = Constants.IntakeConstants.bottomArmKI;
    intakeBottomArmFXConfig.Slot0.kD = Constants.IntakeConstants.bottomArmKD;

    /* Intake Arm MotionMagic Values */
    intakeTopArmFXConfig.MotionMagic.MotionMagicAcceleration = Constants.IntakeConstants.intakeArmMotionMagicAccel;
    intakeTopArmFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakeConstants.intakeArmMotionMagicCruise;
    intakeBottomArmFXConfig.MotionMagic.MotionMagicAcceleration = Constants.IntakeConstants.intakeArmMotionMagicAccel;
    intakeBottomArmFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakeConstants.intakeArmMotionMagicCruise;

    /* Intake Arm Ratios */
    intakeTopArmFXConfig.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.topArmRatio;
    intakeBottomArmFXConfig.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.bottomArmRatio;
    
    /* Climber Winch Values */
    climberWinchFXConfiguration.Slot0.kP = Constants.ClimberConstants.winchKP;
    climberWinchFXConfiguration.Slot0.kI = Constants.ClimberConstants.winchKI;
    climberWinchFXConfiguration.Slot0.kD = Constants.ClimberConstants.winchKD;
    climberWinchFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimberConstants.winchMotionMagicCruise;
    climberWinchFXConfiguration.MotionMagic.MotionMagicAcceleration = Constants.ClimberConstants.winchMotionMagicAccel; 
  }
}