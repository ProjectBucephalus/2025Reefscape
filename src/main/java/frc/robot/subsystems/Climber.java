package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.IDConstants;
import frc.robot.util.Conversions;
import frc.robot.util.SD;

public class Climber extends SubsystemBase
{
  /* Declarations of the motor controller */
  private TalonFX m_ClimberWinch;
  private ClimberStatus status;

  /* Declarations of all the motion magic variables */
  private final MotionMagicVoltage motionMagic;
  private double speed;
  private TalonFXConfiguration config = CTREConfigs.climberWinchFXConfig;

  public enum ClimberStatus 
  {
    ACTIVE,
    STOW,
    MANUAL,
    CLIMB
  };

  public Climber() 
  { 
    m_ClimberWinch = new TalonFX(IDConstants.climberWinchMotorID);
    m_ClimberWinch.getConfigurator().apply(config);
    m_ClimberWinch.setPosition(ClimberConstants.stowWinchPos);
    
    motionMagic = new MotionMagicVoltage(0);

    setStatus(ClimberStatus.STOW);
  }
  
  private void setStatus(ClimberStatus newStatus)
  {
    
    if (newStatus == ClimberStatus.CLIMB)
    {
      if (status == ClimberStatus.STOW)
      { // Active state has required protections for leaving Stow position
        status = ClimberStatus.ACTIVE;
      }
      else
      {
        m_ClimberWinch.getConfigurator().apply(config.MotionMagic.withMotionMagicCruiseVelocity(ClimberConstants.winchClimbCruise));
        status = newStatus;
      }
    }
    else
    {
      m_ClimberWinch.getConfigurator().apply(config.MotionMagic.withMotionMagicCruiseVelocity(ClimberConstants.winchDefaultCruise));
      status = newStatus;
    }
  }

  public Command setStatusCommand(ClimberStatus status)
  {
    return Commands.runOnce(() -> this.setStatus(status), this);
  }

  public boolean isUnlocked()
    {return status != ClimberStatus.STOW;}

  public boolean manualOveride(double motorSpeed)
  {
    speed = motorSpeed;
    setStatus(ClimberStatus.MANUAL);
    return true;
  }

  @Override
  public void periodic()
  {
    SD.CLIMBER_POS.put(m_ClimberWinch.getPosition().getValueAsDouble());

    switch (status)
    {
      case STOW:
        m_ClimberWinch.setControl(motionMagic.withPosition(ClimberConstants.stowWinchPos));
        SD.CLIMBER_TARGET.put(ClimberConstants.stowWinchPos);
        break;

      case ACTIVE:
        if (RobotContainer.s_Diffector.climbSafe())
        {
          m_ClimberWinch.setControl(motionMagic.withPosition(ClimberConstants.activeWinchPos));
          SD.CLIMBER_TARGET.put(ClimberConstants.activeWinchPos);
        }
        else
        {
          m_ClimberWinch.setControl(motionMagic.withPosition(m_ClimberWinch.getPosition().getValueAsDouble()));
        }
        break;

      case CLIMB:
        if (RobotContainer.s_Diffector.climbReady())
        {
          double adjustedClimberPos = ClimberConstants.climbWinchPos;
          
          if (SD.OVERRIDE.get()) 
          {
            adjustedClimberPos += RobotContainer.s_Swerve.getPigeon2().getPitch().getValueAsDouble() * ClimberConstants.winchBalanceScalar;
            adjustedClimberPos = Conversions.clamp(adjustedClimberPos, ClimberConstants.climbActiveInnerLimit, ClimberConstants.climbActiveOuterLimit);
          }

          m_ClimberWinch.setControl(motionMagic.withPosition(adjustedClimberPos));
          SD.CLIMBER_TARGET.put(adjustedClimberPos);
        }
        else
        {
          m_ClimberWinch.setControl(motionMagic.withPosition(m_ClimberWinch.getPosition().getValueAsDouble()));
        }
      
        break;

      case MANUAL:
        if (speed != 0)
          {m_ClimberWinch.set(speed * ClimberConstants.manualScale);}
        else
          {m_ClimberWinch.setControl(motionMagic.withPosition(m_ClimberWinch.getPosition().getValueAsDouble()));}
        break;
    }
  }
}
