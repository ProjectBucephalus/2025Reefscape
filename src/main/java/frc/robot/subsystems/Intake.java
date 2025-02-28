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
import frc.robot.constants.IDConstants;

/**
 * Intake subsystem, handling the intake wheels, and intake arms.
 * 
 * @author 5985
 * @depricated
 */
public class Intake extends SubsystemBase 
{

  /* Declarations of all the motor controllers */
  private TalonFX m_AlgaeIntake;
  private TalonFX m_AlgaeArm;
  private IntakeStatus status;
  boolean touchedAlgae;

  private DigitalInput algaeIntakeBeamBreak;
  private boolean algae;

  /* Declarations of all the motion magic variables */
  private final MotionMagicVoltage motionMagic;
  private double algaeArmTarget;
  
  /* Enum representing the status the intake is in
   * (Spinning inwards for a coral, spinning inwards for an algae, position for when the robot is climbing,
   * Position for before the robot intakes, stowing the coral or algae, transferring the coral to the robot's scorer,
   * Transferring the algae to the robot's scorer)
   */
  public enum IntakeStatus 
  {
    INTAKE_ALGAE,
    EJECT_ALGAE,
    CLIMBING,
    STAND_BY,
    STOWED,
    TRANSFER_ALGAE,
    TESTING
  };
  
  public Intake() 
  { 
    status = IntakeStatus.TESTING;

    m_AlgaeIntake = new TalonFX(IDConstants.algaeIntakeRollerID);
    m_AlgaeArm = new TalonFX(IDConstants.algaeIntakeArmID);

    m_AlgaeArm.getConfigurator().apply(CTREConfigs.algaeIntakeArmFXConfig);
    
    algaeArmTarget = 0;

    motionMagic = new MotionMagicVoltage(0);

    algaeIntakeBeamBreak    = new DigitalInput(IDConstants.algaeIntakeDIO);
  }

  public double getTopArmAngle()
    {return m_AlgaeArm.getPosition().getValueAsDouble() * 360;}

  /** 
   * Set speed of the Algae intake motor
   * 
   * @param speed Algae intake motor speed [-1..1]
   */
  public void setAlgaeIntakeSpeed(double speed)
    {m_AlgaeIntake.set(speed);}

  /**
   * Set the angle of the Algae arm 
   * 
   * @param newAlgaeTarget Algae arm angle, degrees clockwise
   */
   public void setAlgaeArmTarget(double newAlgaeTarget)
    {algaeArmTarget = newAlgaeTarget;}

  public boolean getAlgaeBeamBreakState()
    {return algaeIntakeBeamBreak.get();}

  /**
   * Sets the speeds to the intake and position to the arms
   * 
   * @param status Enum corresponds to the intake motor speeds and
   * arms position
   */
  public void setIntakeStatus(IntakeStatus status){}
    //{this.status = status;}

  /**
   * Gets the target the Algae arm wants to go to
   * 
   * @return AlgaeArmTarget current value
   */
  public double getAlgaeArmTarget()
    {return algaeArmTarget;}

  public boolean isAlgaeStowed()
    {return Constants.IntakeConstants.algaeStowedLowThreshold < getTopArmAngle() && getTopArmAngle() < Constants.IntakeConstants.algaeStowedHighThreshold;}
  
  public boolean getAlgaeState()
    {return algae;}

  public boolean climbReady()
    {return (m_AlgaeArm.getPosition()).getValueAsDouble() > Constants.IntakeConstants.algaeClimbingArmTarget;}
    
  @Override
  public void periodic()  
  {
    if (!getAlgaeBeamBreakState() && status == IntakeStatus.INTAKE_ALGAE)
      {touchedAlgae = true;}
    if (getAlgaeBeamBreakState() && touchedAlgae)
    {
      setIntakeStatus(IntakeStatus.TRANSFER_ALGAE);
      touchedAlgae = false;
    }

    switch (status)
    {
      case TESTING:
        break;

      case INTAKE_ALGAE:
        setAlgaeIntakeSpeed(Constants.IntakeConstants.algaeIntakeMotorSpeed);
        setAlgaeArmTarget(Constants.IntakeConstants.topAlgaeIntakeArmTarget);
        break;

      case EJECT_ALGAE:
        setAlgaeIntakeSpeed(Constants.IntakeConstants.algaeEjectMotorSpeed);
        setAlgaeArmTarget(Constants.IntakeConstants.topAlgaeEjectArmTarget);
        break;

      case CLIMBING:
        setAlgaeIntakeSpeed(Constants.IntakeConstants.climbingIntakeMotorSpeed);
        setAlgaeArmTarget(Constants.IntakeConstants.algaeClimbingArmTarget);
        break;

      case STAND_BY:
        setAlgaeIntakeSpeed(Constants.IntakeConstants.standByMotorSpeed);
        setAlgaeArmTarget(Constants.IntakeConstants.topStandByArmTarget);
        break;

      case STOWED:
        setAlgaeIntakeSpeed(Constants.IntakeConstants.stowedMotorSpeed);
        setAlgaeArmTarget(Constants.IntakeConstants.topStowedArmTarget);
        break;

      case TRANSFER_ALGAE:
        setAlgaeIntakeSpeed(Constants.IntakeConstants.algaeTransferMotorSpeed);
        setAlgaeArmTarget(Constants.IntakeConstants.topAlgaeTransferArmTarget);
        break;
    }

    if (getTopArmAngle() >= algaeArmTarget) 
    {
      // Runs arm with PID slot for spring behaviour
      m_AlgaeArm.setControl(motionMagic.withPosition(algaeArmTarget / 360).withSlot(0));
    }
    else
    {
      if (RobotContainer.s_Diffector.atPosition())
      {
        // Runs arm with PID slot for hardstop behaviour
        m_AlgaeArm.setControl(motionMagic.withPosition(algaeArmTarget / 360).withSlot(1));
      }
       
    }  
  }
}

