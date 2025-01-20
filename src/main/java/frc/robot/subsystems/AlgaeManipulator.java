// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

public class AlgaeManipulator extends SubsystemBase {

  private TalonFX algaeMotor;

  private DigitalInput algaeBeamBreak;

  private algaeManipulatorStatus algaeStatus;

  public enum algaeManipulatorStatus
  {
    INTAKE,
    HOLDING,
    NET,
    PROCESSOR,
    EMPTY
  }

  public AlgaeManipulator() 
  {
    algaeMotor = new TalonFX(Constants.GamePiecesManipulator.ManipulatorIDs.algaeMotorID);
    algaeBeamBreak = new DigitalInput(Constants.GamePiecesManipulator.ManipulatorIDs.algaeBeamBreakDigitalInput);
  }

  private void setAlgaeIntakeSpeed(double Speed)
  {
    algaeMotor.set(Speed);
  }

  private boolean getAlgaeBeamBreakState()
  {
    return algaeBeamBreak.get();
  }
  
  public void setAlgaeManipulatorStatus(algaeManipulatorStatus status)
  {
    algaeStatus = status;
  }


  @Override
  public void periodic() 
  {
    switch(algaeStatus)
    {
      case INTAKE:
        setAlgaeIntakeSpeed(Constants.GamePiecesManipulator.AlgaeManipulator.algaeManipulatorIntakeSpeed);
        break;

      case HOLDING:
        if (getAlgaeBeamBreakState() == true) 
        {
          algaeMotor.setVoltage(Constants.GamePiecesManipulator.AlgaeManipulator.algaeManipulatorHoldingSpeed);      
        }
  
        else if (getAlgaeBeamBreakState() == false)
        {
          setAlgaeIntakeSpeed(0);
        }
      break;

      case NET:
        setAlgaeIntakeSpeed(Constants.GamePiecesManipulator.AlgaeManipulator.algaeManipulatorNetSpeed);
        break;

      case PROCESSOR:
        setAlgaeIntakeSpeed(Constants.GamePiecesManipulator.AlgaeManipulator.algaeManipulatorProcessorSpeed);
        break;

      case EMPTY:
        setAlgaeIntakeSpeed(Constants.GamePiecesManipulator.AlgaeManipulator.algaeManipulatorEmptySpeed);
        break;
    }
  }
}
