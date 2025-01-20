// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

public class CoralManipulator extends SubsystemBase {

  private TalonFX coralMotor;

  private DigitalInput coralBeamBreak1;
  private DigitalInput coralBeamBreak2;

  private coralManipulatorStatus coralStatus;

  // private boolean useBeamBreak1 = false;
  // private boolean useBeamBreak2 = false;


  public enum coralManipulatorStatus
  {
    INTAKE,
    DELIVERY,
    HOLDING
  }

  public CoralManipulator() 
  {
    coralMotor = new TalonFX(Constants.GamePiecesManipulator.ManipulatorIDs.coralMotorID);
    coralBeamBreak1 = new DigitalInput(Constants.GamePiecesManipulator.ManipulatorIDs.coralBeamBreak1DigitalInput);
    coralBeamBreak2 = new DigitalInput(Constants.GamePiecesManipulator.ManipulatorIDs.coralBeamBreak2DigitalInput);
  }

  private void setCoralIntakeSpeed(double Speed)
  {
    coralMotor.set(Speed);
  }

  private boolean getCoralBeamBreak1State()
  {
    return coralBeamBreak1.get();
  }

  private boolean getCoralBeamBreak2State()
  {
    return coralBeamBreak2.get();
  }

  public void setCoralManipulatorStatus(coralManipulatorStatus Status)
  {
    coralStatus = Status;
  }
  

  @Override
  public void periodic() 
  {
    switch(coralStatus)
    {
      case INTAKE:
        setCoralIntakeSpeed(Constants.GamePiecesManipulator.CoralManipulator.coralManipulatorIntakeSpeed);
        break;

      case DELIVERY:
        setCoralIntakeSpeed(Constants.GamePiecesManipulator.CoralManipulator.coralManipulatorDeliverySpeed);
        if (Diffector.getArmPos() >= 0 && Diffector.getArmPos() <= 90)
        {
          setCoralIntakeSpeed(Constants.GamePiecesManipulator.CoralManipulator.coralManipulatorDeliverySpeed);
        } else if 
        break;

      case HOLDING:
        if (getCoralBeamBreak1State() && getCoralBeamBreak2State())
        {
          setCoralIntakeSpeed(0);
        }
        
        else if (getCoralBeamBreak1State() && !getCoralBeamBreak2State())
        {
          setCoralIntakeSpeed(Constants.GamePiecesManipulator.CoralManipulator.coralManipulatorHoldingSpeed);
        }
        
        else if (!getCoralBeamBreak1State() && getCoralBeamBreak2State()) 
        {
          setCoralIntakeSpeed(-Constants.GamePiecesManipulator.CoralManipulator.coralManipulatorHoldingSpeed);
        }
        
        else if (!getCoralBeamBreak1State() && !getCoralBeamBreak1State()) 
        {
          setCoralIntakeSpeed(0);
        }
        break;
    }
  }
}
