// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;

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
    mTopIntake = new TalonFX(Constants.Intake.MotorID.mTopIntakeID);
    mBottomIntake = new TalonFX(Constants.Intake.MotorID.mBottomIntakeID);
    mTopArm = new TalonFX(Constants.Intake.MotorID.mTopArmID);
    mBottomArm = new TalonFX(Constants.Intake.MotorID.mBottomArmID);
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
   * @param pos Top arm angle [0..90..180..270]
   */
  private void setTopArmPos(double pos)
  {
    mTopArm.set(pos);
  };    

  /**
   * Set the angle of the bottom arm 
   * 
   * @param pos Bottom arm angle [0..90..180.207]
   */
  private void setBottomArmPos(double pos)
  {
    mBottomArm.set(pos);
  };


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
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mCoralIntakeMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mCoralIntakeMotorSpeed);
      setTopArmPos(Constants.Intake.ArmPosition.mTopCoralIntakeArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomCoralIntakeArmPos);
      break;

      case INTAKE_ALGAE:
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mAlgaeIntakeMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mAlgaeIntakeMotorSpeed);
      setTopArmPos(Constants.Intake.ArmPosition.mTopAlgaeIntakeArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomAlgaeIntakeArmPos);
      break;

      case CLIMBING:
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mClimbingIntakeMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mClimbingIntakeMotorSpeed);
      setTopArmPos(Constants.Intake.ArmPosition.mTopClimbingArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomClimbingArmPos);
      break;

      case STAND_BY:
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mStandByMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mStandByMotorSpeed);
      setTopArmPos(Constants.Intake.ArmPosition.mTopStandByArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomStandByArmPos);
      break;

      case STOWED:
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mStowedMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mStowedMotorSpeed);
      setTopArmPos(Constants.Intake.ArmPosition.mTopStowedArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomStowedArmPos);
      break;

      case TRANSFER_CORAL:
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mCoralTransferMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mCoralTransferMotorSpeed);
      setTopArmPos(Constants.Intake.ArmPosition.mTopCoralTransferArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomCoralTransferArmPos);
      break;

      case TRANSFER_ALGAE:
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mAlgaeTransferMotorSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mAlgaeTransferMotorSpeed);
      setTopArmPos(Constants.Intake.ArmPosition.mTopAlgaeTransferArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomAlgaeTransferArmPos);
      break;
    }
  };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

