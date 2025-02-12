// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
  private VictorSPX coralMotor;

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
    coralMotor = new VictorSPX(Constants.GamePiecesManipulator.coralMotorID);
    
    coralStatus = CoralManipulatorStatus.INTAKE;
  }

  public CoralManipulatorStatus getStatus()
    {return coralStatus; }

  /**
   * Sets the coral manipulator motor speed
   * 
   * @param speed Coral manipulator motor speed, positive to eject [-1..1]
   */
  public void setCoralManipulatorSpeed(double speed)
    {coralMotor.set(VictorSPXControlMode.PercentOutput, speed); }

  public void setCoralManipulatorStatus(CoralManipulatorStatus status)
    {coralStatus = status;}

  @Override
  public void periodic() 
  {
    RobotContainer.coral = !RobotContainer.s_Canifier.coralManiPortSensor() || !RobotContainer.s_Canifier.coralManiStbdSensor();

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
        if (RobotContainer.s_Canifier.coralManiPortSensor() && RobotContainer.s_Canifier.coralManiStbdSensor())
          {setCoralManipulatorSpeed(0);} 
        else if (RobotContainer.s_Canifier.coralManiPortSensor() && !RobotContainer.s_Canifier.coralManiStbdSensor())
          {setCoralManipulatorSpeed(Constants.GamePiecesManipulator.coralManipulatorIntakeSpeed);} 
        else if (!RobotContainer.s_Canifier.coralManiPortSensor() && RobotContainer.s_Canifier.coralManiStbdSensor()) 
          {setCoralManipulatorSpeed(-Constants.GamePiecesManipulator.coralManipulatorIntakeSpeed);} 
        else if (!RobotContainer.s_Canifier.coralManiPortSensor() && !RobotContainer.s_Canifier.coralManiStbdSensor()) 
          {coralMotor.set(VictorSPXControlMode.PercentOutput, 0); }
        break;
    }
  }
}
