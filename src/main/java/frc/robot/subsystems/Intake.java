// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;

/**
 * Intake subsystem, handling the intake wheels, and intake arms.
 * 
 * @author 5985
 * @author Sebastian Aiello 
 */
public class Intake extends SubsystemBase {

  /* Declarations of all the motor controllers */
  private TalonFX m_AlgaeIntake;
  private TalonFX m_CoralIntake;
  private TalonFX m_AlgaeArm;
  private TalonFX m_CoralArm;

  /* Declarations of all the motion magic variables */
  private final MotionMagicVoltage motionMagic;
  private double AlgaeArmTarget;
  private double CoralArmTarget;
  private final double AlgaeArmRatio;
  private final double CoralArmRatio;
  private double[] armTargets = new double[2];
  
  /* Enum representing the status the intake is in
   * (Spinning inwards for a coral, spinning inwards for an algae, position for when the robot is climbing,
   * Position for before the robot intakes, stowing the coral or algae, transferring the coral to the robot's scorer,
   * Transferring the algae to the robot's scorer)
   */

  public enum IntakeStatus 
  {
    INTAKE_CORAL,
    INTAKE_ALGAE,
    CLIMBING,
    STAND_BY,
    STOWED,
    TRANSFER_CORAL,
    TRANSFER_ALGAE
  };
  
  public Intake() 
  { 
    m_AlgaeIntake = new TalonFX(Constants.Intake.algaeIntakeID);
    m_CoralIntake = new TalonFX(Constants.Intake.coralIntakeID);
    m_AlgaeArm = new TalonFX(Constants.Intake.algaeArmID);
    m_CoralArm = new TalonFX(Constants.Intake.coralArmID);

    m_AlgaeArm.getConfigurator().apply(CTREConfigs.intakeTopArmFXConfig);

    AlgaeArmTarget = 0;
    CoralArmTarget = 0;

    motionMagic = new MotionMagicVoltage(0);
    AlgaeArmRatio = 0;
    CoralArmRatio = 0;
  }

  /** 
   * Set speed of the Algae intake motor
   * 
   * @param speed Algae intake motor speed [-1..1]
   */
  private void setAlgaeIntakeSpeed(double speed)
  {
    m_AlgaeIntake.set(speed);
  };

  /** 
   * Set speed of the Coral intake motor
   * 
   * @param speed Coral intake motor speed [-1..1]
   */
  private void setCoralIntakeSpeed(double speed)
  {
    m_CoralIntake.set(speed);
  };

  /**
   * Set the angle of the Algae arm 
   * 
   * @param newAlgaeTarget Algae arm angle, degrees clockwise
   */
   public void setAlgaeArmTarget(double newAlgaeTarget)
   {
     AlgaeArmTarget = newAlgaeTarget;
   }
    

  /**
   * Set the angle of the Coral arm 
   * 
   * @param newCoralTarget Coral arm angle, degrees clockwise
   */
  public void setCoralArmTarget(double newCoralTarget)
  {
    CoralArmTarget = newCoralTarget;
  }

  /**
   * Calculates the arms target angles
   * 
   * @param newAlgaeTarget Algae arm's target angle
   * @param newCoralTarget Coral arms target angle
   */
  private void calculateArmTargets(double newAlgaeTarget, double newCoralTarget)
  {
    armTargets[0] = newAlgaeTarget / AlgaeArmRatio;
    armTargets[1] = newCoralTarget / CoralArmRatio;
  }


  /**
   * Sets the speeds to the intake and position to the arms
   * 
   * @param Status Enum corresponds to the intake motor speeds and
   * arms position
   */
  public void setIntakeStatus(IntakeStatus Status)
  {
    switch (Status)
    {

      case INTAKE_CORAL:
      setAlgaeIntakeSpeed(Constants.Intake.coralIntakeMotorSpeed);
      setCoralIntakeSpeed(Constants.Intake.coralIntakeMotorSpeed);
      setAlgaeArmTarget(Constants.Intake.topCoralIntakeArmTarget);
      setCoralArmTarget(Constants.Intake.bottomCoralIntakeArmTarget);
      break;

      case INTAKE_ALGAE:
      setAlgaeIntakeSpeed(Constants.Intake.algaeIntakeMotorSpeed);
      setCoralIntakeSpeed(Constants.Intake.algaeIntakeMotorSpeed);
      setAlgaeArmTarget(Constants.Intake.topAlgaeIntakeArmTarget);
      setCoralArmTarget(Constants.Intake.bottomAlgaeIntakeArmTarget);
      break;

      case CLIMBING:
      setAlgaeIntakeSpeed(Constants.Intake.climbingIntakeMotorSpeed);
      setCoralIntakeSpeed(Constants.Intake.climbingIntakeMotorSpeed);
      setAlgaeArmTarget(Constants.Intake.topClimbingArmTarget);
      setCoralArmTarget(Constants.Intake.bottomClimbingArmTarget);
      break;

      case STAND_BY:
      setAlgaeIntakeSpeed(Constants.Intake.standByMotorSpeed);
      setCoralIntakeSpeed(Constants.Intake.standByMotorSpeed);
      setAlgaeArmTarget(Constants.Intake.topStandByArmTarget);
      setCoralArmTarget(Constants.Intake.bottomStandByArmTarget);
      break;

      case STOWED:
      setAlgaeIntakeSpeed(Constants.Intake.stowedMotorSpeed);
      setCoralIntakeSpeed(Constants.Intake.stowedMotorSpeed);
      setAlgaeArmTarget(Constants.Intake.topStowedArmTarget);
      setCoralArmTarget(Constants.Intake.bottomStowedArmTarget);
      break;

      case TRANSFER_CORAL:
      setAlgaeIntakeSpeed(Constants.Intake.coralTransferMotorSpeed);
      setCoralIntakeSpeed(Constants.Intake.coralTransferMotorSpeed);
      setAlgaeArmTarget(Constants.Intake.topCoralTransferArmTarget);
      setCoralArmTarget(Constants.Intake.bottomCoralTransferArmTarget);
      break;

      case TRANSFER_ALGAE:
      setAlgaeIntakeSpeed(Constants.Intake.algaeTransferMotorSpeed);
      setCoralIntakeSpeed(Constants.Intake.algaeTransferMotorSpeed);
      setAlgaeArmTarget(Constants.Intake.topAlgaeTransferArmTarget);
      setCoralArmTarget(Constants.Intake.bottomAlgaeTransferArmTarget);
      break;
    }
  }

  /**
   * Gets the target the Algae arm wants to go to
   * 
   * @return AlgaeArmTarget current value
   */
  public double getAlgaeArmTarget()
  {
    return AlgaeArmTarget;
  }

  /**
   * Gets the target the Coral arm want to go to
   * 
   * @return CoralArmTarget current value
   */
  public double getCoralArmTarget()
  {
    return CoralArmTarget;
  }

  public boolean isCoralStowed()
  {
    return m_CoralIntake.getPosition().getValueAsDouble() <= Constants.Intake.coralStowedThreshold;
  } 

  public boolean isAlgaeStowed()
  {
    return m_AlgaeIntake.getPosition().getValueAsDouble() <= Constants.Intake.algaeStowedThreshold;
  }

  @Override
  public void periodic()  
  {
    calculateArmTargets(AlgaeArmTarget, CoralArmTarget);
    m_AlgaeArm.setControl(motionMagic.withPosition(armTargets[0]));
    m_CoralArm.setControl(motionMagic.withPosition(armTargets[1]));
  }
}

