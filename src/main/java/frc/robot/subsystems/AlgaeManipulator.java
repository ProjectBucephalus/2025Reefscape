// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;

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
    EJECT,
    EMPTY
  }

  public AlgaeManipulator() 
  {
    algaeStatus = AlgaeManipulatorStatus.EMPTY;
    algaeMotor = new TalonFX(IDConstants.algaeManipulatorID);
  }

  public void setStatus(AlgaeManipulatorStatus status)
    {algaeStatus = status;}

  public Command setStatusCommand(AlgaeManipulatorStatus status)
    {return runOnce(() -> setStatus(status));}

  public AlgaeManipulatorStatus getStatus()
    {return algaeStatus;}

  @Override
  public void periodic() 
  {
    RobotContainer.algae = algaeMotor.getTorqueCurrent().getValueAsDouble() >= Constants.GamePiecesManipulator.algaeHeldCurrent;

    switch(algaeStatus)
    {
      case INTAKE:
        algaeMotor.set(Constants.GamePiecesManipulator.algaeIntakeSpeed);

        if (RobotContainer.algae) 
          {algaeStatus = AlgaeManipulatorStatus.HOLDING;}
        break;

      case HOLDING:
        if (RobotContainer.algae) 
          {algaeMotor.set(Constants.GamePiecesManipulator.algaeHoldingSpeed);} 

        else
          {algaeStatus = AlgaeManipulatorStatus.EMPTY;}
        break;

      case EJECT:
        double armPos = RobotContainer.s_Diffector.getRelativeRotation();

        if (armPos > 90 + Constants.DiffectorConstants.algaeEjectSpeedAngleThreshold && armPos <= 270 - Constants.DiffectorConstants.algaeEjectSpeedAngleThreshold)
          {algaeMotor.set(Constants.GamePiecesManipulator.algaeNetSpeed);}
        else
          {algaeMotor.set(Constants.GamePiecesManipulator.algaeProcessorSpeed);}
        break;

      case EMPTY:
        algaeMotor.set(0);

        if (RobotContainer.algae) 
          {algaeStatus = AlgaeManipulatorStatus.HOLDING;}
        break;
    }
  }
}
