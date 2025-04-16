package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;
import frc.robot.constants.Constants.DiffectorConstants.Presets;
import frc.robot.constants.Constants.Manipulators;
import frc.robot.constants.DiffectorGeometry;
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
  private TalonFX m_Coral;

  /**
   * Enum representing the status this manipulator is in
   * (Keeps speed at zero while intaking,
   * While delivering if the arm position is less than 180 degrees, then the speed is set to positive
   * And if the arm position is more than 180 degrees, then the speed is set to negitive, 
   * And while holding, if one of the beam breaks don't see the coral, then coral moves to that beam break till they both see them)
   */
  public enum Status {INTAKE, DELIVERY_LEFT, DELIVERY_RIGHT, DEFAULT, DELIVERY_SMART, WIGGLE}

  /* Declaration of the enum variable */
  private Status status;

  private int wiggleCounter = 0;
  private boolean portWiggle = false;

  /** For use in switch cases with smart directionality */
  private double speed;
  private double armAngle;

  public CoralManipulator() 
  {
    setStatus(Status.DEFAULT);
    m_Coral = new TalonFX(IDConstants.coralMotorID);
  }

  public Status getStatus()
    {return status;}

  public void setStatus(Status newStatus)
  {
    if (newStatus == Status.WIGGLE && status != Status.WIGGLE)
    {
      wiggleCounter = 0;
      portWiggle = false;
    }
    status = newStatus;
  }

  public Command setStatusCommand(Status status)
    {return runOnce(() -> setStatus(status)).withName("SetCoralStatus");}

  @Override
  public void periodic() 
  {
    RobotContainer.coral = !RobotContainer.io_Canifier.coralPortSensor() || !RobotContainer.io_Canifier.coralStbdSensor();
    SmartDashboard.putBoolean("Coral", RobotContainer.coral);

    switch(status)
    {
      case WIGGLE:
        if (wiggleCounter < Constants.Manipulators.coralWiggleCount)
        {
          if (RobotContainer.io_Canifier.coralPortSensor() && !RobotContainer.io_Canifier.coralStbdSensor())
          {
            m_Coral.set(Manipulators.coralHoldingSpeed);
            if (!portWiggle)
            {
              portWiggle = true;
              wiggleCounter++;
            }
          }

          else if (!RobotContainer.io_Canifier.coralPortSensor() && RobotContainer.io_Canifier.coralStbdSensor()) 
          {
            m_Coral.set(-Manipulators.coralHoldingSpeed);
            if (portWiggle)
            {
              portWiggle = false;
              wiggleCounter++;
            }
          } 

          else if (!RobotContainer.io_Canifier.coralPortSensor() && !RobotContainer.io_Canifier.coralStbdSensor() && (m_Coral.get() == 0)) 
            {m_Coral.set(-Manipulators.coralHoldingSpeed);}
        }
        else 
        {
          setStatus(Status.DEFAULT);
          wiggleCounter = 0;
        }
        break;

      case INTAKE:
        if (RobotContainer.coral) 
          {setStatus(Status.WIGGLE);}
        else
        {
          speed = Manipulators.coralHoldingSpeed;
          armAngle = RobotContainer.s_Diffector.getRelativeRotation();
          if (armAngle < 180)
            {speed = -speed;}
          m_Coral.set(speed);
        }
        break;

      case DELIVERY_SMART:
        int nearestReefFace = FieldUtils.getNearestReefFace(RobotContainer.swerveState.Pose.getTranslation());
        armAngle = RobotContainer.s_Diffector.getRelativeRotation();
        double armHeight = RobotContainer.s_Diffector.getElevation();
        speed = 
        MathUtil.isNear(Presets.coral4Position.getZ(), armHeight, DiffectorGeometry.elevationTolerance) ?
        Manipulators.coralLvl4DeliverySpeed
        :
        Manipulators.coralDeliverySpeed;

        if ((nearestReefFace == 5 || nearestReefFace == 6) ^ (armAngle > 90 && armAngle <= 270)) 
          {speed = -speed;}

        m_Coral.set(speed);
        break;

      case DELIVERY_LEFT:
      case DELIVERY_RIGHT:
        speed = Manipulators.coralDeliverySpeed;

        armAngle = RobotContainer.s_Diffector.getRelativeRotation();
        double robotRotation = Conversions.mod(RobotContainer.swerveState.Pose.getRotation().getDegrees(), 360);

        if 
        (
          status == Status.DELIVERY_RIGHT ^
          (armAngle > 90 && armAngle <= 270) ^
          (
            robotRotation > 90 - Constants.Control.driverVisionTolerance && 
            robotRotation <= 270 + Constants.Control.driverVisionTolerance
          )
        ) 
          {speed = -speed;}
          
        m_Coral.set(speed);
        break;

      case DEFAULT:
        if (RobotContainer.io_Canifier.coralPortSensor() && RobotContainer.io_Canifier.coralStbdSensor())
          {m_Coral.set(0);}

        else if (RobotContainer.io_Canifier.coralPortSensor() && !RobotContainer.io_Canifier.coralStbdSensor())
          {m_Coral.set(Manipulators.coralHoldingSpeed);}

        else if (!RobotContainer.io_Canifier.coralPortSensor() && RobotContainer.io_Canifier.coralStbdSensor()) 
          {m_Coral.set(-Manipulators.coralHoldingSpeed);} 
          
        else if (!RobotContainer.io_Canifier.coralPortSensor() && !RobotContainer.io_Canifier.coralStbdSensor()) 
          {m_Coral.set(0);}
        break;
    }
  }
}
