package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Coral manipulator subsystem, handing the intake, out-take, 
 * and holding of coral for the coral manipulator
 * 
 * @author 5985
 */
public class CoralManipulator extends SubsystemBase 
{
  /* Declaration of the motor controllers */
  private TalonFX motor;

  /**
   * Enum representing the status this manipulator is in
   * (Keeps speed at zero while intaking,
   * While delivering if the arm position is less than 180 degrees, then the speed is set to positive
   * And if the arm position is more than 180 degrees, then the speed is set to negitive, 
   * And while holding, if one of the beam breaks don't see the coral, then coral moves to that beam break till they both see them)
   */
  public enum Status {INTAKE, DELIVERY_LEFT, DELIVERY_RIGHT, DEFAULT, DELIVERY_SMART}

  /* Declaration of the enum variable */
  private Status status;

  /** For use in switch cases with smart directionality */
  private double speed;
  private double armPos;

  public CoralManipulator() 
  {
    status = Status.DEFAULT;
    motor = new TalonFX(IDConstants.coralMotorID);
  }

  public Status getStatus()
    {return status;}

  private void setSpeedFeedforward(double speed)
    {motor.set(speed + Math.sin(Units.degreesToRadians(RobotContainer.diffector.getAngle())) * Constants.Manipulators.coralHoldingkG);}

  public void setStatus(Status newStatus)
    {status = newStatus;}

  public Command setStatusCommand(Status status)
    {return runOnce(() -> setStatus(status));}

  public Command scoreCommand()
  {
    return 
    Commands.either
    (
      startEnd(null, null), 
      startEnd(() -> setStatus(Status.DELIVERY_SMART), () -> setStatus(Status.DEFAULT)), 
      null
    );
  }

  @Override
  public void periodic() 
  {
    RobotContainer.coral = !RobotContainer.canifier.coralPortSensor() || !RobotContainer.canifier.coralStbdSensor();

    switch(status)
    {
      case INTAKE:
        motor.set(Constants.Manipulators.coralHoldingSpeed);
        
        if (RobotContainer.coral) 
          {status = Status.DEFAULT;}
      break;

      case DELIVERY_SMART:
        int nearestReefFace = FieldUtils.getNearestReefFace(RobotContainer.swerveState.Pose.getTranslation());
        speed = -Constants.Manipulators.coralDeliverySpeed;
        armPos = RobotContainer.diffector.getRelativeRotation();

        if (nearestReefFace == 5 || nearestReefFace == 6) 
        {
          speed = -speed;
        }

        if (armPos > 90 && armPos <= 270)
          {speed = -speed;}

        motor.set(speed);
      break;

      case DELIVERY_LEFT:
      case DELIVERY_RIGHT:
        speed = Constants.Manipulators.coralDeliverySpeed;

        armPos = RobotContainer.diffector.getRelativeRotation();
        double robotRotation = Conversions.mod(RobotContainer.swerveState.Pose.getRotation().getDegrees(), 360);

        if (status == Status.DELIVERY_RIGHT) 
          {speed = -speed;}

        if (armPos > 90 && armPos <= 270)
          {speed = -speed;}

        if (robotRotation > 90 - Constants.Control.driverVisionTolerance && robotRotation <= 270 + Constants.Control.driverVisionTolerance) 
          {speed = -speed;}

        motor.set(speed);
      break;

      case DEFAULT:
        if (RobotContainer.canifier.coralPortSensor() && RobotContainer.canifier.coralStbdSensor())
          {motor.set(0);}

        else if (RobotContainer.canifier.coralPortSensor() && !RobotContainer.canifier.coralStbdSensor())
          {setSpeedFeedforward(Constants.Manipulators.coralHoldingSpeed);}

        else if (!RobotContainer.canifier.coralPortSensor() && RobotContainer.canifier.coralStbdSensor()) 
          {setSpeedFeedforward(-Constants.Manipulators.coralHoldingSpeed);} 
          
        else if (!RobotContainer.canifier.coralPortSensor() && !RobotContainer.canifier.coralStbdSensor()) 
          {setSpeedFeedforward(0);}
      break;
    }
  }
}
