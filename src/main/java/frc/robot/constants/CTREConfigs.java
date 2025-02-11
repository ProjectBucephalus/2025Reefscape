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

    /* Intake Top Arm PID Config, Spring Behaviour */
    intakeTopArmFXConfig.Slot0.kP = Constants.Intake.topArmSpringKP;
    intakeTopArmFXConfig.Slot0.kI = Constants.Intake.topArmSpringKI;
    intakeTopArmFXConfig.Slot0.kD = Constants.Intake.topArmSpringKD;
    intakeTopArmFXConfig.Slot0.kS = Constants.Intake.topArmKS;
    intakeTopArmFXConfig.Slot0.kG = -Constants.Intake.topArmKG;

    /* Intake Top Arm PID Config, Stop Behaviour */
    intakeTopArmFXConfig.Slot1.kP = Constants.Intake.topArmStopKP;
    intakeTopArmFXConfig.Slot1.kI = Constants.Intake.topArmStopKI;
    intakeTopArmFXConfig.Slot1.kD = Constants.Intake.topArmStopKD;
    intakeTopArmFXConfig.Slot1.kS = Constants.Intake.topArmKS;
    intakeTopArmFXConfig.Slot1.kG = Constants.Intake.topArmKG;

    /* Intake Bottom Arm PID Config */
    intakeBottomArmFXConfig.Slot0.kP = Constants.Intake.bottomArmKP;
    intakeBottomArmFXConfig.Slot0.kI = Constants.Intake.bottomArmKI;
    intakeBottomArmFXConfig.Slot0.kD = Constants.Intake.bottomArmKD;

    /* Intake Arm MotionMagic Values */
    intakeTopArmFXConfig.MotionMagic.MotionMagicAcceleration = Constants.Intake.intakeArmMotionMagicAccel;
    intakeTopArmFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.intakeArmMotionMagicCruise;
    intakeBottomArmFXConfig.MotionMagic.MotionMagicAcceleration = Constants.Intake.intakeArmMotionMagicAccel;
    intakeBottomArmFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.intakeArmMotionMagicCruise;

    /* Intake Arm Ratios */
    intakeTopArmFXConfig.Feedback.SensorToMechanismRatio = Constants.Intake.topArmRatio;
    intakeBottomArmFXConfig.Feedback.SensorToMechanismRatio = Constants.Intake.bottomArmRatio;
    
    /* Climber Winch Values */
    climberWinchFXConfiguration.Slot0.kP = Constants.Climber.winchKP;
    climberWinchFXConfiguration.Slot0.kI = Constants.Climber.winchKI;
    climberWinchFXConfiguration.Slot0.kD = Constants.Climber.winchKD;
    climberWinchFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.winchMotionMagicCruise;
    climberWinchFXConfiguration.MotionMagic.MotionMagicAcceleration = Constants.Climber.winchMotionMagicAccel; 
  }
}