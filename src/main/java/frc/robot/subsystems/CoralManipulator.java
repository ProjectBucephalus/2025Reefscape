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
  private TalonFX coralMotor;

  /**
   * Enum representing the status this manipulator is in
   * (Keeps speed at zero while intaking,
   * While delivering if the arm position is less than 180 degrees, then the speed is set to positive
   * And if the arm position is more than 180 degrees, then the speed is set to negitive, 
   * And while holding, if one of the beam breaks don't see the coral, then coral moves to that beam break till they both see them)
   */
  public enum CoralStatus {INTAKE, DELIVERY_LEFT, DELIVERY_RIGHT, DEFAULT, DELIVERY_SMART}

  /* Declaration of the enum variable */
  private CoralStatus coralStatus;

  /** For use in switch cases with smart directionality */
  private double speed;
  private double armPos;

  public CoralManipulator() 
  {
    coralStatus = CoralStatus.DEFAULT;
    coralMotor = new TalonFX(IDConstants.coralManipulatorID);
  }

  public CoralStatus getStatus()
    {return coralStatus;}

  private void setSpeedFeedforward(double speed)
    {coralMotor.set(speed + Math.sin(Units.degreesToRadians(RobotContainer.s_Diffector.getAngle())) * Constants.GamePiecesManipulator.coralHoldingkG);}

  public void setStatus(CoralStatus status)
    {coralStatus = status;}

  public Command setStatusCommand(CoralStatus status)
    {return runOnce(() -> setStatus(status));}

  public Command scoreCommand()
  {
    return 
    Commands.either
    (
      startEnd(null, null), 
      startEnd(() -> setStatus(CoralStatus.DELIVERY_SMART), () -> setStatus(CoralStatus.DEFAULT)), 
      null
    );
  }

  @Override
  public void periodic() 
  {
    RobotContainer.coral = !RobotContainer.s_Canifier.coralManiPortSensor() || !RobotContainer.s_Canifier.coralManiStbdSensor();

    switch(coralStatus)
    {
      case INTAKE:
        coralMotor.set(Constants.GamePiecesManipulator.coralHoldingSpeed);
        
        if (RobotContainer.coral) 
          {coralStatus = CoralStatus.DEFAULT;}
      break;

      case DELIVERY_SMART:
        int nearestReefFace = FieldUtils.getNearestReefFace(RobotContainer.swerveState.Pose.getTranslation());
        speed = -Constants.GamePiecesManipulator.coralDeliverySpeed;
        armPos = RobotContainer.s_Diffector.getRelativeRotation();

        if (nearestReefFace == 5 || nearestReefFace == 6) 
        {
          speed = -speed;
        }

        if (armPos > 90 && armPos <= 270)
          {speed = -speed;}

        coralMotor.set(speed);
      break;

      case DELIVERY_LEFT:
      case DELIVERY_RIGHT:
        speed = Constants.GamePiecesManipulator.coralDeliverySpeed;

        armPos = RobotContainer.s_Diffector.getRelativeRotation();
        double robotRotation = Conversions.mod(RobotContainer.swerveState.Pose.getRotation().getDegrees(), 360);

        if (coralStatus == CoralStatus.DELIVERY_RIGHT) 
          {speed = -speed;}

        if (armPos > 90 && armPos <= 270)
          {speed = -speed;}

        if (robotRotation > 90 - Constants.Control.driverVisionTolerance && robotRotation <= 270 + Constants.Control.driverVisionTolerance) 
          {speed = -speed;}

        coralMotor.set(speed);
      break;

      case DEFAULT:
        if (RobotContainer.s_Canifier.coralManiPortSensor() && RobotContainer.s_Canifier.coralManiStbdSensor())
          {coralMotor.set(0);}

        else if (RobotContainer.s_Canifier.coralManiPortSensor() && !RobotContainer.s_Canifier.coralManiStbdSensor())
          {setSpeedFeedforward(Constants.GamePiecesManipulator.coralHoldingSpeed);}

        else if (!RobotContainer.s_Canifier.coralManiPortSensor() && RobotContainer.s_Canifier.coralManiStbdSensor()) 
          {setSpeedFeedforward(-Constants.GamePiecesManipulator.coralHoldingSpeed);} 
          
        else if (!RobotContainer.s_Canifier.coralManiPortSensor() && !RobotContainer.s_Canifier.coralManiStbdSensor()) 
          {setSpeedFeedforward(0);}
      break;
    }
  }
}
