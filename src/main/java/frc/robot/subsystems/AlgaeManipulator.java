// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Algae manipulator Subsystem, handling the intake and out-take
 * Of the algae for the algae manipulator.
 * 
 * @author 5985
 * @author Sebastian Aiello
 */
public class AlgaeManipulator extends SubsystemBase 
{

  /* Declaration of the motor controllers */
  private TalonFX algaeMotor;

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
    algaeStatus = AlgaeManipulatorStatus.EMPTY;
    algaeMotor = new TalonFX(Constants.GamePiecesManipulator.algaeMotorID);
    algaeStatus = AlgaeManipulatorStatus.EMPTY;
  }

  /**
   * Sets the speed of the algae manipulator motor
   * 
   * @param Speed Algae manipulator motor speed, positive to eject [-1..1]
   */
  public void setAlgaeManipulatorSpeed(double Speed)
    {algaeMotor.set(Speed);}

  public void setAlgaeManipulatorStatus(AlgaeManipulatorStatus status)
    {algaeStatus = status;}

  public AlgaeManipulatorStatus getStatus()
    {return algaeStatus;}

  @Override
  public void periodic() 
  {
    RobotContainer.algae = !RobotContainer.s_Canifier.algaeManiSensor();

    switch(algaeStatus)
    {
      case INTAKE:
        setAlgaeManipulatorSpeed(Constants.GamePiecesManipulator.algaeManipulatorIntakeSpeed);
        if (RobotContainer.algae) 
          {algaeStatus = AlgaeManipulatorStatus.HOLDING;}
        break;

      case HOLDING:
        if (RobotContainer.algae) 
        {
          algaeMotor.setVoltage(Constants.GamePiecesManipulator.algaeManipulatorHoldingVoltage);      
        } 
        else
          {algaeStatus = AlgaeManipulatorStatus.EMPTY;}
        break;

      case NET:
        setAlgaeManipulatorSpeed(Constants.GamePiecesManipulator.algaeManipulatorNetSpeed);
        if (!RobotContainer.algae) 
          {algaeStatus = AlgaeManipulatorStatus.EMPTY;}
        break;

      case PROCESSOR:
        setAlgaeManipulatorSpeed(Constants.GamePiecesManipulator.algaeManipulatorProcessorSpeed);
        if (!RobotContainer.algae) 
          {algaeStatus = AlgaeManipulatorStatus.EMPTY;}
        break;

      case EMPTY:
        setAlgaeManipulatorSpeed(Constants.GamePiecesManipulator.algaeManipulatorEmptySpeed);
        if (RobotContainer.algae) 
          {algaeStatus = AlgaeManipulatorStatus.HOLDING;}
        break;
    }
  }
}
