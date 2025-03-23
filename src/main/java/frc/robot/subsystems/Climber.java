package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.IDConstants;
import frc.robot.constants.MechanismConstants.ClimberConfigs;
import frc.robot.util.Conversions;
import frc.robot.util.SD;

public class Climber extends SubsystemBase
{
  /* Declarations of the motor controller */
  private TalonFX m_Climber;
  private Status status;

  /* Declarations of all the motion magic variables */
  private final MotionMagicVoltage motionMagic;
  private double speed;
  private TalonFXConfiguration motorConfig = ClimberConfigs.climberMotorConfig;
  private boolean climbLocked = true;

  public enum Status 
  {
    ACTIVE,
    STOW,
    MANUAL,
    CLIMB,
    HOLD
  };

  public Climber() 
  { 
    m_Climber = new TalonFX(IDConstants.climberWinchMotorID);
    m_Climber.getConfigurator().apply(motorConfig);
    m_Climber.setPosition(ClimberConstants.stowWinchPos);
    
    motionMagic = new MotionMagicVoltage(0);

    setStatus(Status.STOW);

    SD.CLIMB_OVERRIDE.init();
  }
  
  private void setStatus(Status newStatus)
  {
    
    if (newStatus == Status.CLIMB)
    {
      if (status == Status.STOW)
      { // Active state has required protections for leaving Stow position
        status = Status.ACTIVE;
      }
      else
      {
        m_Climber.getConfigurator().apply(motorConfig.MotionMagic.withMotionMagicCruiseVelocity(ClimberConfigs.winchClimbCruise));
        status = newStatus;
      }
    }
    else
    {
      m_Climber.getConfigurator().apply(motorConfig.MotionMagic.withMotionMagicCruiseVelocity(ClimberConfigs.winchDefaultCruise));
      status = newStatus;
    }
  }

  public Command setStatusCommand(Status status)
  {
    return runOnce(() -> setStatus(status)).withName("SetClimberStatus");
  }

  public boolean climbReady()
    {return m_Climber.getPosition().getValueAsDouble() >= ClimberConstants.prepareWinchPos;}

  public boolean armSafe()
    {return m_Climber.getPosition().getValueAsDouble() >= ClimberConstants.safeWinchPos;}  

  public boolean offGround()
    {return m_Climber.getPosition().getValueAsDouble() >= ClimberConstants.offGroundPos;}  

  public boolean manualOveride(double motorSpeed)
  {
    speed = motorSpeed;
    setStatus(Status.MANUAL);
    return true;
  }

  public void unlockClimb()
    {climbLocked = false;}

  @Override
  public void periodic()
  {
    SD.CLIMBER_POS.put(m_Climber.getPosition().getValueAsDouble());
    if (SD.CLIMB_OVERRIDE.get()) {climbLocked = false;}

    switch (status)
    {
      case STOW:
        m_Climber.setControl(motionMagic.withPosition(ClimberConstants.stowWinchPos));
        SD.CLIMBER_TARGET.put(ClimberConstants.stowWinchPos);
        break;

      case ACTIVE:
        if (RobotContainer.s_Diffector.climbSafe())
        {
          m_Climber.setControl(motionMagic.withPosition(ClimberConstants.prepareWinchPos));
          SD.CLIMBER_TARGET.put(ClimberConstants.prepareWinchPos);
        }
        break;

      case CLIMB:
        if (RobotContainer.s_Diffector.climbReady())
        {
          double adjustedClimberPos = ClimberConstants.climbWinchPos;
          
          if (SD.OVERRIDE.get()) 
          {
            adjustedClimberPos += RobotContainer.s_Swerve.getPigeon2().getPitch().getValueAsDouble() * ClimberConfigs.winchBalanceScalar;
            adjustedClimberPos = Conversions.clamp(adjustedClimberPos, ClimberConstants.climbActiveInnerLimit, ClimberConstants.climbActiveOuterLimit);
          }

          m_Climber.setControl(motionMagic.withPosition(adjustedClimberPos));
          SD.CLIMBER_TARGET.put(adjustedClimberPos);
        }
        break;

      case MANUAL:
        if 
        (
          (
            (speed > 0 && m_Climber.getPosition().getValueAsDouble() <= 1.05 * ClimberConstants.prepareWinchPos) || 
            (speed < 0 && m_Climber.getPosition().getValueAsDouble() >= 0) || 
            (speed != 0 && SD.CLIMB_OVERRIDE.get())
          ) 
          && !climbLocked
        )
          {m_Climber.set(speed * Control.manualClimberScale);}
        else
        {
          m_Climber.setControl(motionMagic.withPosition(m_Climber.getPosition().getValueAsDouble()));
          status = Status.HOLD;
        }
        break;
      
      case HOLD:
        break;
    }
    SmartDashboard.putString("Climber State", status.name());
  }
}
