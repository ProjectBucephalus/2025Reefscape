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
  private TalonFX motor;

  /* Declaration of the enum variable */
  private Status status;

  /**
   * Enum representing the status this manipulator is in
   * (Spins inwards to pull the algae in, gives a small amount of voltage to the motors to hold the algae in,
   * Spins outwards at full speed to shoot at the net, spins outwards to shoot in the processor,
   * And does nothing while empty)
   */
  public enum Status
  {
    INTAKE,
    HOLDING,
    EJECT,
    EMPTY
  }

  public AlgaeManipulator() 
  {
    status = Status.EMPTY;
    motor = new TalonFX(IDConstants.algaeMotorID);
  }

  public void setStatus(Status newStatus)
    {status = newStatus;}

  public Command setStatusCommand(Status status)
    {return runOnce(() -> setStatus(status));}

  public Status getStatus()
    {return status;}

  @Override
  public void periodic() 
  {
    RobotContainer.algae = motor.getTorqueCurrent().getValueAsDouble() >= Constants.Manipulators.algaeHeldCurrent;

    switch(status)
    {
      case INTAKE:
        motor.set(Constants.Manipulators.algaeIntakeSpeed);

        if (RobotContainer.algae) 
          {status = Status.HOLDING;}
        break;

      case HOLDING:
        if (RobotContainer.algae) 
          {motor.set(Constants.Manipulators.algaeHoldingSpeed);} 

        else
          {status = Status.EMPTY;}
        break;

      case EJECT:
        double armPos = RobotContainer.diffector.getRelativeRotation();

        if (armPos > 90 + Constants.DiffectorConstants.algaeEjectSpeedAngleThreshold && armPos <= 270 - Constants.DiffectorConstants.algaeEjectSpeedAngleThreshold)
          {motor.set(Constants.Manipulators.algaeNetSpeed);}
        else
          {motor.set(Constants.Manipulators.algaeProcessorSpeed);}
        break;

      case EMPTY:
        motor.set(0);

        if (RobotContainer.algae) 
          {status = Status.HOLDING;}
        break;
    }
  }
}
