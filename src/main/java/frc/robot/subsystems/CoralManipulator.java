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
    DEFAULT
  }

  public CoralManipulator() 
  {
    coralStatus = CoralManipulatorStatus.DEFAULT;
    coralMotor = new TalonFX(Constants.GamePiecesManipulator.coralMotorID);
    coralBeamBreak1 = new DigitalInput(Constants.GamePiecesManipulator.coralManipulatorDIO1);
    coralBeamBreak2 = new DigitalInput(Constants.GamePiecesManipulator.coralManipulatorDIO2);
    
    coralStatus = CoralManipulatorStatus.INTAKE;
  }

  public CoralManipulatorStatus getStatus()
    {return coralStatus; }

  /**
   * Sets the coral manipulator motor speed
   * 
   * @param Speed Coral manipulator motor speed, positive to eject [-1..1]
   */
  public void setCoralManipulatorSpeed(double Speed)
    {coralMotor.set(Speed); }

  public void setCoralManipulatorStatus(CoralManipulatorStatus Status)
    {coralStatus = Status; }

  @Override
  public void periodic() 
  {
    RobotContainer.coral = !coralBeamBreak2.get() || !coralBeamBreak1.get();

    switch(coralStatus)
    {
      case INTAKE:
        setCoralManipulatorSpeed(Constants.GamePiecesManipulator.coralManipulatorIntakeSpeed);
        if (RobotContainer.coral) 
          {coralStatus = CoralManipulatorStatus.DEFAULT; }
        break;

      case DELIVERY:
        if (RobotContainer.s_Diffector.getArmPos() % 360 <= 180)
          {setCoralManipulatorSpeed(Constants.GamePiecesManipulator.coralManipulatorDeliverySpeed);} 
        else
          {setCoralManipulatorSpeed(-Constants.GamePiecesManipulator.coralManipulatorDeliverySpeed); }
        if (!RobotContainer.coral) 
          {coralStatus = CoralManipulatorStatus.DEFAULT; }
        break;

      case DEFAULT:
        if (coralBeamBreak1.get() && coralBeamBreak2.get())
          {setCoralManipulatorSpeed(0);} 
        else if (coralBeamBreak1.get() && !coralBeamBreak2.get())
          {setCoralManipulatorSpeed(Constants.GamePiecesManipulator.coralManipulatorIntakeSpeed);} 
        else if (!coralBeamBreak1.get() && coralBeamBreak2.get()) 
          {setCoralManipulatorSpeed(-Constants.GamePiecesManipulator.coralManipulatorIntakeSpeed);} 
        else if (!coralBeamBreak1.get() && !coralBeamBreak2.get()) 
          {coralMotor.setVoltage(Constants.GamePiecesManipulator.coralManipulatorHoldingVoltage); }
        break;
    }
  }
}
