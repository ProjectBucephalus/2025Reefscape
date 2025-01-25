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
  private TalonFX m_TopIntake;
  private TalonFX m_BottomIntake;
  private TalonFX m_TopArm;
  private TalonFX m_BottomArm;

  /* Declarations of all the motion magic variables */
  private final MotionMagicVoltage motionMagic;
  private double topArmTarget;
  private double bottomArmTarget;
  
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
    m_TopIntake = new TalonFX(Constants.Intake.topIntakeID);
    m_BottomIntake = new TalonFX(Constants.Intake.bottomIntakeID);
    m_TopArm = new TalonFX(Constants.Intake.topArmID);
    m_BottomArm = new TalonFX(Constants.Intake.bottomArmID);

    m_TopArm.getConfigurator().apply(CTREConfigs.intakeTopArmFXConfig);
    m_BottomArm.getConfigurator().apply(CTREConfigs.intakeBottomArmFXConfig);

    topArmTarget = 0;
    bottomArmTarget = 0;

    motionMagic = new MotionMagicVoltage(0);
  }

  public double getTopArmAngle()
  {
    return m_TopArm.getPosition().getValueAsDouble() * 360;
  }

  public double getBottomArmAngle()
  {
    return m_BottomArm.getPosition().getValueAsDouble() * 360;
  }

  /** 
   * Set speed of the top intake motor
   * 
   * @param speed Top intake motor speed [-1..1]
   */
  private void setTopIntakeSpeed(double speed)
  {
    m_TopIntake.set(speed);
  };

  /** 
   * Set speed of the bottom intake motor
   * 
   * @param speed Bottom intake motor speed [-1..1]
   */
  private void setBottomIntakeSpeed(double speed)
  {
    m_BottomIntake.set(speed);
  };

  /**
   * Set the angle of the top arm 
   * 
   * @param newTopTarget Top arm angle, degrees clockwise
   */
   public void setTopArmTarget(double newTopTarget)
   {
     topArmTarget = newTopTarget;
   }

  /**
   * Set the angle of the bottom arm 
   * 
   * @param newBottomTarget Bottom arm angle, degrees clockwise
   */
  public void setBottomArmTarget(double newBottomTarget)
  {
    bottomArmTarget = newBottomTarget;
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
      setTopIntakeSpeed(Constants.Intake.coralIntakeMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.coralIntakeMotorSpeed);
      setTopArmTarget(Constants.Intake.topCoralIntakeArmTarget);
      setBottomArmTarget(Constants.Intake.bottomCoralIntakeArmTarget);
      break;

      case INTAKE_ALGAE:
      setTopIntakeSpeed(Constants.Intake.algaeIntakeMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.algaeIntakeMotorSpeed);
      setTopArmTarget(Constants.Intake.topAlgaeIntakeArmTarget);
      setBottomArmTarget(Constants.Intake.bottomAlgaeIntakeArmTarget);
      break;

      case CLIMBING:
      setTopIntakeSpeed(Constants.Intake.climbingIntakeMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.climbingIntakeMotorSpeed);
      setTopArmTarget(Constants.Intake.topClimbingArmTarget);
      setBottomArmTarget(Constants.Intake.bottomClimbingArmTarget);
      break;

      case STAND_BY:
      setTopIntakeSpeed(Constants.Intake.standByMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.standByMotorSpeed);
      setTopArmTarget(Constants.Intake.topStandByArmTarget);
      setBottomArmTarget(Constants.Intake.bottomStandByArmTarget);
      break;

      case STOWED:
      setTopIntakeSpeed(Constants.Intake.stowedMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.stowedMotorSpeed);
      setTopArmTarget(Constants.Intake.topStowedArmTarget);
      setBottomArmTarget(Constants.Intake.bottomStowedArmTarget);
      break;

      case TRANSFER_CORAL:
      setTopIntakeSpeed(Constants.Intake.coralTransferMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.coralTransferMotorSpeed);
      setTopArmTarget(Constants.Intake.topCoralTransferArmTarget);
      setBottomArmTarget(Constants.Intake.bottomCoralTransferArmTarget);
      break;

      case TRANSFER_ALGAE:
      setTopIntakeSpeed(Constants.Intake.algaeTransferMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.algaeTransferMotorSpeed);
      setTopArmTarget(Constants.Intake.topAlgaeTransferArmTarget);
      setBottomArmTarget(Constants.Intake.bottomAlgaeTransferArmTarget);
      break;
    }
  }

  /**
   * Gets the target the top arm wants to go to
   * 
   * @return TopArmTarget current value
   */
  public double getTopArmTarget()
  {
    return topArmTarget;
  }

  /**
   * Gets the target the bottom arm want to go to
   * 
   * @return BottomArmTarget current value
   */
  public double getBottomArmTarget()
  {
    return bottomArmTarget;
  }

  @Override
  public void periodic()  
  {
    if (getTopArmAngle() >= topArmTarget) 
    {
      // Runs arm with PID slot for spring behaviour
      m_TopArm.setControl(motionMagic.withPosition(topArmTarget / 360).withSlot(0));
    }
    else
    {
      // Runs arm with PID slot for hardstop behaviour
      m_TopArm.setControl(motionMagic.withPosition(topArmTarget / 360).withSlot(1));
    }

    m_BottomArm.setControl(motionMagic.withPosition(bottomArmTarget / 360));
  }
}

