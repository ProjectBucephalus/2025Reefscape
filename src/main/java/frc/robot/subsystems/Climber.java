package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;
import frc.robot.util.Conversions;

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
    m_ClimberWinch.setPosition(Constants.ClimberConstants.stowWinchPos);
    
    motionMagic = new MotionMagicVoltage(0);

    setStatus(ClimberStatus.STOW);
  }
  
  private void setStatus(ClimberStatus newStatus)
  {
    status = newStatus;
    if (newStatus == ClimberStatus.CLIMB)
    {
      m_ClimberWinch.getConfigurator().apply(config.MotionMagic.withMotionMagicCruiseVelocity(Constants.ClimberConstants.winchClimbCruise));
    }
    else
    {
      m_ClimberWinch.getConfigurator().apply(config.MotionMagic.withMotionMagicCruiseVelocity(Constants.ClimberConstants.winchDefaultCruise));
    }
  }

  public Command setStatusCommand(ClimberStatus status)
  {
    return Commands.runOnce(() -> this.setStatus(status), this);
  }

  public boolean isUnlocked()
    {return status == ClimberStatus.ACTIVE;}

  public boolean manualOveride(double motorSpeed)
  {
    speed = motorSpeed;
    setStatus(ClimberStatus.MANUAL);
    return true;
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Climber Position", m_ClimberWinch.getPosition().getValueAsDouble());

    switch (status)
    {
      case STOW:
        m_ClimberWinch.setControl(motionMagic.withPosition(Constants.ClimberConstants.stowWinchPos));
        SmartDashboard.putNumber("Climber Target", Constants.ClimberConstants.stowWinchPos);
        break;

      case ACTIVE:
        m_ClimberWinch.setControl(motionMagic.withPosition(Constants.ClimberConstants.activeWinchPos));
        SmartDashboard.putNumber("Climber Target", Constants.ClimberConstants.activeWinchPos);
        break;

      case CLIMB:
        double adjustedClimberPos = Constants.ClimberConstants.climbWinchPos;

        if (SmartDashboard.getBoolean("OVERIDE MODE", false)) 
        {
          adjustedClimberPos += RobotContainer.s_Swerve.getPigeon2().getPitch().getValueAsDouble() * Constants.ClimberConstants.winchBalanceScalar;
          adjustedClimberPos = Conversions.clamp(adjustedClimberPos, Constants.ClimberConstants.climbWinchInnerLimit, Constants.ClimberConstants.climbWinchOuterLimit);
        }

        m_ClimberWinch.setControl(motionMagic.withPosition(adjustedClimberPos));
        SmartDashboard.putNumber("Climber Target", adjustedClimberPos);
        break;

      case MANUAL:
        if (speed != 0)
          {m_ClimberWinch.set(speed * Constants.ClimberConstants.manualScale);}
        else
          {m_ClimberWinch.setControl(motionMagic.withPosition(m_ClimberWinch.getPosition().getValueAsDouble()));}
        break;
    }
  }
}
