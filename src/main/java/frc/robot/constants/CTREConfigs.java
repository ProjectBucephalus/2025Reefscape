package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class CTREConfigs 
{
  public static final TalonFXConfiguration algaeIntakeArmFXConfig = new TalonFXConfiguration();
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

    /* Algae Intake Arm PID Config, Spring Behaviour */
    algaeIntakeArmFXConfig.Slot0.kP = Constants.IntakeConstants.algaeIntakeArmSpringKP;
    algaeIntakeArmFXConfig.Slot0.kI = Constants.IntakeConstants.algaeIntakeArmSpringKI;
    algaeIntakeArmFXConfig.Slot0.kD = Constants.IntakeConstants.algaeIntakeArmSpringKD;
    algaeIntakeArmFXConfig.Slot0.kS = Constants.IntakeConstants.algaeIntakeArmKS;
    algaeIntakeArmFXConfig.Slot0.kG = -Constants.IntakeConstants.algaeIntakeArmKG;

    /* Algae Intake Arm PID Config, Stop Behaviour */
    algaeIntakeArmFXConfig.Slot1.kP = Constants.IntakeConstants.topArmStopKP;
    algaeIntakeArmFXConfig.Slot1.kI = Constants.IntakeConstants.topArmStopKI;
    algaeIntakeArmFXConfig.Slot1.kD = Constants.IntakeConstants.topArmStopKD;
    algaeIntakeArmFXConfig.Slot1.kS = Constants.IntakeConstants.algaeIntakeArmKS;
    algaeIntakeArmFXConfig.Slot1.kG = Constants.IntakeConstants.algaeIntakeArmKG;

    algaeIntakeArmFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /* Intake Arm MotionMagic Values */
    algaeIntakeArmFXConfig.MotionMagic.MotionMagicAcceleration = Constants.IntakeConstants.intakeArmMotionMagicAccel;
    algaeIntakeArmFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakeConstants.intakeArmMotionMagicCruise;

    /* Intake Arm Ratios */
    algaeIntakeArmFXConfig.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.algaeIntakeArmRatio;
  }
}