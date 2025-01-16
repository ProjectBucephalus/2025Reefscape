// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  private TalonFX mTopIntake;
  private TalonFX mBottomIntake;
  private TalonFX mTopArm;
  private TalonFX mBottomArm;
  
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


  private void setTopIntakeSpeed(double speed)
  {
    mTopIntake.set(speed);
  };

  private void setBottomIntakeSpeed(double speed)
  {
    mBottomIntake.set(speed);
  };

  private void setTopArmPos(double pos)
  {
    mTopArm.set(pos);
  };    

  private void setBottomArmPos(double pos)
  {
    mBottomArm.set(pos);
  };

  public void setIntakeStatus(IntakeStatus Status)
  {
    switch (Status)
    {

      //TODO: Get proper values of the speeds and angles

      case INTAKE_CORAL:
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mIntakeSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mIntakeSpeed);
      setTopArmPos(Constants.Intake.ArmPosition.mTopCoralIntakeArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomCoralIntakeArmPos);
      break;

      case INTAKE_ALGAE:
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mIntakeSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mIntakeSpeed);
      setTopArmPos(Constants.Intake.ArmPosition.mTopAlgaeIntakeArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomAlgaeIntakeArmPos);
      break;

      case CLIMBING:
      setTopIntakeSpeed(0);
      setBottomIntakeSpeed(0);
      setTopArmPos(Constants.Intake.ArmPosition.mTopClimbingArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomClimbingArmPos);
      break;

      case STAND_BY:
      setTopIntakeSpeed(0);
      setBottomIntakeSpeed(0);
      setTopArmPos(Constants.Intake.ArmPosition.mTopStandByArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomStandByArmPos);
      break;

      case STOWED:
      setTopIntakeSpeed(0);
      setBottomIntakeSpeed(0);
      setTopArmPos(Constants.Intake.ArmPosition.mTopStowedArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomStowedArmPos);
      break;

      case TRANSFER_CORAL:
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mTransferSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mTransferSpeed);
      setTopArmPos(Constants.Intake.ArmPosition.mTopCoralTransferArmPos);
      setBottomArmPos(Constants.Intake.ArmPosition.mBottomCoralTransferArmPos);
      break;

      case TRANSFER_ALGAE:
      setTopIntakeSpeed(Constants.Intake.MotorSpeeds.mTransferSpeed);
      setBottomIntakeSpeed(Constants.Intake.MotorSpeeds.mTransferSpeed);
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

