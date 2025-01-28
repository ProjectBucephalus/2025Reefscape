// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Coral manipulator subsystem, handing the intake, out-take,
 * And hold of coral for the coral manipulator
 * 
 * @author 5985
 * @author Sebastian Aiello
 */
public class CoralManipulator extends SubsystemBase {

  /* Declaration of the motor controllers */
  private TalonFX coralMotor;

  /* Declaration of the beam break sensor */
  private DigitalInput coralBeamBreak1;
  private DigitalInput coralBeamBreak2;

  /* Declaration of the enum variable */
  private CoralManipulatorStatus coralStatus;

  /**
   * Enum representing the status this manipulator is in
   * (Keeps speed at zero while intaking,
   * While delivering if the arm position is less than 180 degrees, then the speed is set to positive
   * And if the arm position is more than 180 degrees, then the speed is set to negitive, 
   * And while holding, if one of the beam breaks don't see the coral, then coral moves to that beam break till they both see them)
   */
  public enum CoralManipulatorStatus
  {
    INTAKE,
    DELIVERY,
    HOLDING
  }

  public CoralManipulator() 
  {
    coralMotor = new TalonFX(Constants.GamePiecesManipulator.coralMotorID);
    coralBeamBreak1 = new DigitalInput(Constants.GamePiecesManipulator.coralBeamBreak1DigitalInput);
    coralBeamBreak2 = new DigitalInput(Constants.GamePiecesManipulator.coralBeamBreak2DigitalInput);
  }

  /**
   * Sets the coral manipulator motor speed
   * 
   * @param Speed Coral manipulator motor speed, positive to eject [-1..1]
   */
  public void setCoralManipulatorSpeed(double Speed)
  {
    coralMotor.set(Speed);
  }

  /**
   * Gets the 1st beam break current state
   * 
   * @return 1st CoralBeamBreak current state, false when object present?
   */
  public boolean getCoralBeamBreak1State()
  {
    return coralBeamBreak1.get();
  }

  /**
   * Gets the 2nd beam break current state
   * 
   * @return 2nd CoralBeamBreak current state, false when object present?
   */
  public boolean getCoralBeamBreak2State()
  {
    return coralBeamBreak2.get();
  }

  public void setCoralManipulatorStatus(CoralManipulatorStatus Status)
  {
    coralStatus = Status;
  }

  @Override
  public void periodic() 
  {
    switch(coralStatus)
    {
      case INTAKE:
        setCoralManipulatorSpeed(Constants.GamePiecesManipulator.coralManipulatorIntakeSpeed);
        break;

      case DELIVERY:
        if (Diffector.getArmPos() % 360 <= 180)
        {
          setCoralManipulatorSpeed(Constants.GamePiecesManipulator.coralManipulatorDeliverySpeed);
        } else
        {
          setCoralManipulatorSpeed(-Constants.GamePiecesManipulator.coralManipulatorDeliverySpeed);
        }
        break;

      case HOLDING:
        if (getCoralBeamBreak1State() && getCoralBeamBreak2State())
        {
          setCoralManipulatorSpeed(0);
        } 
        else if (getCoralBeamBreak1State() && !getCoralBeamBreak2State())
        {
          setCoralManipulatorSpeed(Constants.GamePiecesManipulator.coralManipulatorHoldingSpeed);
        } 
        else if (!getCoralBeamBreak1State() && getCoralBeamBreak2State()) 
        {
          setCoralManipulatorSpeed(-Constants.GamePiecesManipulator.coralManipulatorHoldingSpeed);
        } 
        else if (!getCoralBeamBreak1State() && !getCoralBeamBreak1State()) 
        {
          setCoralManipulatorSpeed(Constants.GamePiecesManipulator.coralManipulatorIntakeSpeed);
        }
        break;
    }
  }
}
