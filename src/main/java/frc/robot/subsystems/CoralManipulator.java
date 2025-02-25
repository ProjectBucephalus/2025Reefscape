// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;
import frc.robot.util.Conversions;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Coral manipulator subsystem, handing the intake, out-take,
 * And hold of coral for the coral manipulator
 * 
 * @author 5985
 * @author Sebastian Aiello
 */
public class CoralManipulator extends SubsystemBase 
{
  /* Declaration of the motor controllers */
  private TalonFX coralMotor;

  /**
   * Enum representing the status this manipulator is in
   * (Keeps speed at zero while intaking,
   * While delivering if the arm position is less than 180 degrees, then the speed is set to positive
   * And if the arm position is more than 180 degrees, then the speed is set to negitive, 
   * And while holding, if one of the beam breaks don't see the coral, then coral moves to that beam break till they both see them)
   */
  public enum CoralManipulatorStatus {INTAKE, DELIVERY_LEFT, DELIVERY_RIGHT, DEFAULT}

    /* Declaration of the enum variable */
    private CoralManipulatorStatus coralStatus;

  public CoralManipulator() 
  {
    coralStatus = CoralManipulatorStatus.DEFAULT;
    coralMotor = new TalonFX(IDConstants.coralManipulatorID);
  }

  public CoralManipulatorStatus getStatus()
    {return coralStatus;}

  /**
   * Sets the coral manipulator motor speed
   * 
   * @param speed Coral manipulator motor speed, positive to eject [-1..1]
   */
  private void setCoralManipulatorSpeed(double speed)
    {coralMotor.set(speed);}

  private void setCoralManipulatorSpeedFeedforward(double speed)
  {
    coralMotor.set(speed + Math.sin(Units.degreesToRadians(RobotContainer.s_Diffector.getAngle())) * Constants.GamePiecesManipulator.coralHoldingkG);
  }

  public void setCoralManipulatorStatus(CoralManipulatorStatus status)
    {coralStatus = status;}

  @Override
  public void periodic() 
  {
    RobotContainer.coral = !RobotContainer.s_Canifier.coralManiPortSensor() || !RobotContainer.s_Canifier.coralManiStbdSensor();

    switch(coralStatus)
    {
      case INTAKE:
        setCoralManipulatorSpeed(Constants.GamePiecesManipulator.coralManipulatorHoldingSpeed);
        
        if (RobotContainer.coral) 
          {coralStatus = CoralManipulatorStatus.DEFAULT;}
        break;

      case DELIVERY_LEFT:
      case DELIVERY_RIGHT:
        double speed = Constants.GamePiecesManipulator.coralManipulatorDeliverySpeed;

        double armPos = RobotContainer.s_Diffector.getRelativeRotation();
        double robotPos = Conversions.mod(RobotContainer.swerveState.Pose.getRotation().getDegrees(), 360);

        if (coralStatus == CoralManipulatorStatus.DELIVERY_LEFT) 
          {speed = -speed;}
        if (armPos > 90 && armPos <= 270)
          {speed = -speed;}
        if (robotPos > 90 + Constants.Control.driverVisionTolerance && robotPos <= 270 - Constants.Control.driverVisionTolerance) 
          {speed = -speed;}

        setCoralManipulatorSpeed(speed);

        if (!RobotContainer.coral) 
          {coralStatus = CoralManipulatorStatus.DEFAULT;}
          break;

      case DEFAULT: // TODO Loop overrun
        if (RobotContainer.s_Canifier.coralManiPortSensor() && RobotContainer.s_Canifier.coralManiStbdSensor())
        {
          setCoralManipulatorSpeed(0);
        } 
        else if (RobotContainer.s_Canifier.coralManiPortSensor() && !RobotContainer.s_Canifier.coralManiStbdSensor())
        {
          setCoralManipulatorSpeedFeedforward(Constants.GamePiecesManipulator.coralManipulatorHoldingSpeed);
        } 
        else if (!RobotContainer.s_Canifier.coralManiPortSensor() && RobotContainer.s_Canifier.coralManiStbdSensor()) 
        {
          setCoralManipulatorSpeedFeedforward(-Constants.GamePiecesManipulator.coralManipulatorHoldingSpeed);
        } 
        else if (!RobotContainer.s_Canifier.coralManiPortSensor() && !RobotContainer.s_Canifier.coralManiStbdSensor()) 
        {
          setCoralManipulatorSpeedFeedforward(0);
        }
        break;
    }
  }
}
