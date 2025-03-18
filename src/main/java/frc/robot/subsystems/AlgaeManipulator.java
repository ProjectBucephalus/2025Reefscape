// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.DiffectorGeometry;
import frc.robot.constants.IDConstants;
import frc.robot.util.SD;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Algae manipulator Subsystem, handling the intake and out-take
 * Of the algae for the algae manipulator.
 * 
 * @author 5985
 */
public class AlgaeManipulator extends SubsystemBase 
{

  /* Declaration of the motor controllers */
  private TalonFX m_Algae;

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
    MANUAL_INTAKE,
    INTAKE,
    HOLDING,
    EJECT,
    EMPTY
  }

  public AlgaeManipulator() 
  {
    status = Status.EMPTY;
    m_Algae = new TalonFX(IDConstants.algaeMotorID);
    SD.IO_ALGAE_HOLD.init();
  }

  public void setStatus(Status newStatus)
    {status = newStatus;}

  public Command setStatusCommand(Status status)
    {return runOnce(() -> setStatus(status)).withName("SetAlgaeStatus");}

  public Status getStatus()
    {return status;}

  @Override
  public void periodic() 
  {
    RobotContainer.algae = 
      Math.abs(m_Algae.getStatorCurrent().getValueAsDouble()) >= Constants.Manipulators.algaeHeldCurrent ||
      (RobotContainer.algae); // && Math.abs(algaeMotor.getStatorCurrent().getValueAsDouble()) >= Constants.GamePiecesManipulator.algaeReleaseCurrent);
    SD.SENSOR_ALGAE.put(RobotContainer.algae);
    SD.STATE_ALGAE.put(status.name());
    SD.SENSOR_ALGAE_CURRENT.put(Math.abs(m_Algae.getStatorCurrent().getValueAsDouble()));
    double algaeHoldingSpeed = SD.IO_ALGAE_HOLD.get();

    switch(status)
    {
      case MANUAL_INTAKE:
        m_Algae.set(Constants.Manipulators.algaeIntakeSpeed);
        break;

      case INTAKE:
        m_Algae.set(Constants.Manipulators.algaeIntakeSpeed);

        if (RobotContainer.algae) 
          {status = Status.HOLDING;}
        break;

      case HOLDING:
        if (RobotContainer.algae) 
          {m_Algae.set(algaeHoldingSpeed);}

        else
          {status = Status.EMPTY;}
        break;

      case EJECT:
        double armPos = RobotContainer.s_Diffector.getRelativeRotation();

        if (armPos > 90 + DiffectorGeometry.algaeEjectSpeedAngleThreshold && armPos <= 270 - DiffectorGeometry.algaeEjectSpeedAngleThreshold)
          {m_Algae.set(Constants.Manipulators.algaeNetSpeed);}
        else
          {m_Algae.set(Constants.Manipulators.algaeProcessorSpeed);}
        break;

      case EMPTY:
        m_Algae.set(0);
        RobotContainer.algae = false;
        break;
    }
  }
}
