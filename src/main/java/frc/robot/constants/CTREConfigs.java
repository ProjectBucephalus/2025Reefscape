package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public static TalonFXConfiguration intakeTopArmFXConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration diffectorFXConfig = new TalonFXConfiguration();

    public CTREConfigs(){
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

        /* Intake Arm PID Config, Spring Behaviour */
        intakeTopArmFXConfig.Slot0.kP = Constants.Intake.armSpringKP;
        intakeTopArmFXConfig.Slot0.kI = Constants.Intake.armSpringKI;
        intakeTopArmFXConfig.Slot0.kD = Constants.Intake.armSpringKD;

        /* Intake Arm PID Config, Stop Behaviour */
        intakeTopArmFXConfig.Slot1.kP = Constants.Intake.armStopKP;
        intakeTopArmFXConfig.Slot1.kI = Constants.Intake.armStopKI;
        intakeTopArmFXConfig.Slot1.kD = Constants.Intake.armStopKD;
    }
}