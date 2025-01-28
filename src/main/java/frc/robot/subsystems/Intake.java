// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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
  private IntakeStatus status;
  boolean touchedAlgae;

  private DigitalInput coralLimitSwitch1;
  private DigitalInput coralLimitSwitch2;
  private DigitalInput algaeBeamBreak;

  /* Declarations of all the motion magic variables */
  private final MotionMagicVoltage motionMagic;
  private double algaeArmTarget;
  private double coralArmTarget;
  
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
    m_CoralArm.getConfigurator().apply(CTREConfigs.intakeBottomArmFXConfig);

    algaeArmTarget = 0;
    coralArmTarget = 0;

    motionMagic = new MotionMagicVoltage(0);
  }

  public double getTopArmAngle()
  {
    return m_AlgaeArm.getPosition().getValueAsDouble() * 360;
  }

  public double getBottomArmAngle()
  {
    return m_CoralArm.getPosition().getValueAsDouble() * 360;
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
  }

  /**
   * Set the angle of the Algae arm 
   * 
   * @param newAlgaeTarget Algae arm angle, degrees clockwise
   */
   public void setAlgaeArmTarget(double newAlgaeTarget)
   {
     algaeArmTarget = newAlgaeTarget;
   }

  /**
   * Set the angle of the Coral arm 
   * 
   * @param newCoralTarget Coral arm angle, degrees clockwise
   */
  public void setCoralArmTarget(double newCoralTarget)
  {
    coralArmTarget = newCoralTarget;
    
  }

  public boolean getCoralSwitch1State()
    {return coralLimitSwitch1.get();}

  public boolean getCoralSwitch2State()
    {return coralLimitSwitch2.get();}

  public boolean getAlgaeBeamBreakState()
    {return algaeBeamBreak.get();}

  /**
   * Sets the speeds to the intake and position to the arms
   * 
   * @param Status Enum corresponds to the intake motor speeds and
   * arms position
   */
  public void setIntakeStatus(IntakeStatus status)
  {
    switch (status)
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
    return algaeArmTarget;
  }

  /**
   * Gets the target the Coral arm want to go to
   * 
   * @return CoralArmTarget current value
   */
  public double getCoralArmTarget()
  {
    return coralArmTarget;
  }

  public boolean isCoralStowed()
  {
    return Constants.Intake.coralStowedLowThreshold < getBottomArmAngle() && getBottomArmAngle() < Constants.Intake.coralStowedHighThreshold;
  } 

  public boolean isAlgaeStowed()
  {
    return Constants.Intake.algaeStowedLowThreshold < getTopArmAngle() && getTopArmAngle() < Constants.Intake.algaeStowedHighThreshold;
  }

  @Override
  public void periodic()  
  {
    if (getCoralSwitch1State() || getCoralSwitch2State() && status == IntakeStatus.INTAKE_CORAL)
    {setIntakeStatus(IntakeStatus.TRANSFER_CORAL);}

    if (!getAlgaeBeamBreakState() && status == IntakeStatus.INTAKE_ALGAE)
    {
      touchedAlgae = true;
    }
    if (getAlgaeBeamBreakState() && touchedAlgae)
    {
      setIntakeStatus(IntakeStatus.TRANSFER_ALGAE);
      touchedAlgae = false;
    }

    if (getTopArmAngle() >= algaeArmTarget) 
    {
      // Runs arm with PID slot for spring behaviour
      m_AlgaeArm.setControl(motionMagic.withPosition(algaeArmTarget / 360).withSlot(0));
    }
    else
    {
      if (RobotContainer.s_Diffector.safeToMoveAlgae())
     {
      // Runs arm with PID slot for hardstop behaviour
      m_AlgaeArm.setControl(motionMagic.withPosition(algaeArmTarget / 360).withSlot(1));
     }
      
    }

    if (RobotContainer.s_Climber.isStowed() && RobotContainer.s_Diffector.safeToMoveCoral())
    {
      m_CoralArm.setControl(motionMagic.withPosition(coralArmTarget / 360));
    }  
  }
}

