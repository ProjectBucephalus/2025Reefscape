// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Intake;

/**
 * Intake subsystem, handling the intake wheels, and intake arms
 * 
 * @author 5985
 * @author Sebastian Aiello 
 */
public class Intake extends SubsystemBase {

  /* Declarations of all the motor controllers */
  private TalonFX mTopIntake;
  private TalonFX mBottomIntake;
  private TalonFX mTopArm;
  private TalonFX mBottomArm;

  private final MotionMagicVoltage motionMagic;
  private double topArmTarget;
  private double bottomArmTarget;
  private final double topArmRatio;
  private final double bottomArmRatio;
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
    mTopIntake = new TalonFX(Intake.Intake.MotorID.mTopIntakeID);
    mBottomIntake = new TalonFX(Intake.Intake.MotorID.mBottomIntakeID);
    mTopArm = new TalonFX(Intake.Intake.MotorID.mTopArmID);
    mBottomArm = new TalonFX(Intake.Intake.MotorID.mBottomArmID);

    topArmTarget = 0;
    bottomArmTarget = 0;

    motionMagic = new MotionMagicVoltage(0);
    topArmRatio = 0;
    bottomArmRatio = 0;
  }

  /** 
   * Set speed of the top intake motor
   * 
   * @param speed Top intake motor speed [-1..0..1]
   */
  private void setTopIntakeSpeed(double speed)
  {
    mTopIntake.set(speed);
  };

  /** 
   * Set speed of the bottom intake motor
   * 
   * @param speed Bottom intake motor speed [-1..0..1]
   */
  private void setBottomIntakeSpeed(double speed)
  {
    mBottomIntake.set(speed);
  };

  /**
   * Set the angle of the top arm 
   * 
   * @param newTopTarget Top arm angle [0..90..180..270]
   */
   public void setTopArmTarget(double newTopTarget)
   {
     topArmTarget = newTopTarget;
   }
    

  /**
   * Set the angle of the bottom arm 
   * 
   * @param newBottomTarget Bottom arm angle [0..90..180.207]
   */
  public void setBottomArmTarget(double newBottomTarget)
  {
    bottomArmTarget = newBottomTarget;
  }

  /**
   * Calculates the arms target angles
   * 
   * @param newTopTarget Top arm's target angle
   * @param newBottomTarget Bottom arms target angle
   */
  private void calculateArmTargets(double newTopTarget, double newBottomTarget)
  {
    armTargets[0] = newTopTarget / topArmRatio;
    armTargets[1] = newBottomTarget / bottomArmRatio;
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
      setTopIntakeSpeed(Intake.Intake.MotorSpeeds.mCoralIntakeMotorSpeed);
      setBottomIntakeSpeed(Intake.Intake.MotorSpeeds.mCoralIntakeMotorSpeed);
      setTopArmTarget(Intake.Intake.ArmPosition.mTopCoralIntakeArmTarget);
      setBottomArmTarget(Intake.Intake.ArmPosition.mBottomCoralIntakeArmTarget);
      break;

      case INTAKE_ALGAE:
      setTopIntakeSpeed(Intake.Intake.MotorSpeeds.mAlgaeIntakeMotorSpeed);
      setBottomIntakeSpeed(Intake.Intake.MotorSpeeds.mAlgaeIntakeMotorSpeed);
      setTopArmTarget(Intake.Intake.ArmPosition.mTopAlgaeIntakeArmTarget);
      setBottomArmTarget(Intake.Intake.ArmPosition.mBottomAlgaeIntakeArmTarget);
      break;

      case CLIMBING:
      setTopIntakeSpeed(Intake.Intake.MotorSpeeds.mClimbingIntakeMotorSpeed);
      setBottomIntakeSpeed(Intake.Intake.MotorSpeeds.mClimbingIntakeMotorSpeed);
      setTopArmTarget(Intake.Intake.ArmPosition.mTopClimbingArmTarget);
      setBottomArmTarget(Intake.Intake.ArmPosition.mBottomClimbingArmTarget);
      break;

      case STAND_BY:
      setTopIntakeSpeed(Intake.Intake.MotorSpeeds.mStandByMotorSpeed);
      setBottomIntakeSpeed(Intake.Intake.MotorSpeeds.mStandByMotorSpeed);
      setTopArmTarget(Intake.Intake.ArmPosition.mTopStandByArmTarget);
      setBottomArmTarget(Intake.Intake.ArmPosition.mBottomStandByArmTarget);
      break;

      case STOWED:
      setTopIntakeSpeed(Intake.Intake.MotorSpeeds.mStowedMotorSpeed);
      setBottomIntakeSpeed(Intake.Intake.MotorSpeeds.mStowedMotorSpeed);
      setTopArmTarget(Intake.Intake.ArmPosition.mTopStowedArmTarget);
      setBottomArmTarget(Intake.Intake.ArmPosition.mBottomStowedArmTarget);
      break;

      case TRANSFER_CORAL:
      setTopIntakeSpeed(Intake.Intake.MotorSpeeds.mCoralTransferMotorSpeed);
      setBottomIntakeSpeed(Intake.Intake.MotorSpeeds.mCoralTransferMotorSpeed);
      setTopArmTarget(Intake.Intake.ArmPosition.mTopCoralTransferArmTarget);
      setBottomArmTarget(Intake.Intake.ArmPosition.mBottomCoralTransferArmTarget);
      break;

      case TRANSFER_ALGAE:
      setTopIntakeSpeed(Intake.Intake.MotorSpeeds.mAlgaeTransferMotorSpeed);
      setBottomIntakeSpeed(Intake.Intake.MotorSpeeds.mAlgaeTransferMotorSpeed);
      setTopArmTarget(Intake.Intake.ArmPosition.mTopAlgaeTransferArmTarget);
      setBottomArmTarget(Intake.Intake.ArmPosition.mBottomAlgaeTransferArmTarget);
      break;
    }
  }

  public double getTopArmTarget()
  {
    return topArmTarget;
  }

  public double getBottomArmTarget()
  {
    return bottomArmTarget;
  }

  @Override
  public void periodic()  
  {
    calculateArmTargets(topArmTarget, bottomArmTarget);
    mTopArm.setControl(motionMagic.withPosition(armTargets[0]));
    mBottomArm.setControl(motionMagic.withPosition(armTargets[1]));
  }
}

