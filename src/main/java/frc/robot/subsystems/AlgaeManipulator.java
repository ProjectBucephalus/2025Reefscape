// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Algae manipulator Subsystem, handling the intake and out-take
 * Of the algae for the algae manipulator.
 * 
 * @author 5985
 * @author Sebastian Aiello
 */
public class AlgaeManipulator extends SubsystemBase {

  /* Declaration of the motor controllers */
  private TalonFX algaeMotor;

  /* Declaration of the beam break sensor */
  private DigitalInput algaeBeamBreak;

  /* Declaration of the enum variable */
  private AlgaeManipulatorStatus algaeStatus;


  /**
   * Enum representing the status this manipulator is in
   * (Spins inwards to pull the algae in, gives a small amount of voltage to the motors to hold the algae in,
   * Spins outwards at full speed to shoot at the net, spins outwards to shoot in the processor,
   * And does nothing while empty)
   */
  public enum AlgaeManipulatorStatus
  {
    INTAKE,
    HOLDING,
    NET,
    PROCESSOR,
    EMPTY
  }

  public AlgaeManipulator() 
  {
    algaeMotor = new TalonFX(Constants.GamePiecesManipulator.algaeMotorID);
    algaeBeamBreak = new DigitalInput(Constants.GamePiecesManipulator.algaeBeamBreakDigitalInput);
  }

  /**
   * Sets the speed of the algae manipulator motor
   * 
   * @param Speed Algae manipulator motor speed, positive to eject [-1..1]
   */
  public void setAlgaeManipulatorSpeed(double Speed)
  {
    algaeMotor.set(Speed);
  }

  /**
   * Gets the beam break current state
   * 
   * @return AlgaeBeamBreak current state, false when object present
   */
  private boolean getAlgaeBeamBreakState()
  {
    return algaeBeamBreak.get();
  }
  
  public void setAlgaeManipulatorStatus(AlgaeManipulatorStatus status)
  {
    algaeStatus = status;
  }


  @Override
  public void periodic() 
  {
    switch(algaeStatus)
    {
      case INTAKE:
        setAlgaeManipulatorSpeed(Constants.GamePiecesManipulator.algaeManipulatorIntakeSpeed);
        if (getAlgaeBeamBreakState()) {
          algaeStatus = AlgaeManipulatorStatus.HOLDING;
        }
        break;

      case HOLDING:
        if (getAlgaeBeamBreakState()) 
        {
          algaeMotor.setVoltage(Constants.GamePiecesManipulator.algaeManipulatorHoldingVoltage);      
        } else
        {
          setAlgaeManipulatorSpeed(0);
        }
        break;

      case NET:
        setAlgaeManipulatorSpeed(Constants.GamePiecesManipulator.algaeManipulatorNetSpeed);
        break;

      case PROCESSOR:
        setAlgaeManipulatorSpeed(Constants.GamePiecesManipulator.algaeManipulatorProcessorSpeed);
        break;

      case EMPTY:
        setAlgaeManipulatorSpeed(Constants.GamePiecesManipulator.algaeManipulatorEmptySpeed);
        break;
    }
  }
}
