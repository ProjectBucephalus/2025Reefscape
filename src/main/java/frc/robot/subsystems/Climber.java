package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;

public class Climber extends SubsystemBase
{
    /* Declarations of all the motor controllers */
  private TalonFX m_ClimberWinch;
  private boolean climbActive;
  private ClimberStatus status;

  //private DigitalInput dio;

  /* Declarations of all the motion magic variables */
  private final MotionMagicVoltage motionMagic;
  private double speed;
  private double manualScale;
  private TalonFXConfiguration config = CTREConfigs.climberWinchFXConfig;
  private boolean climberClearanceFlag = false;

  public enum ClimberStatus 
  {
    ACTIVE,
    STOW,
    MANUAL,
    CLIMB,
    INTAKE //TODO
  };

  public Climber() 
  { 
    m_ClimberWinch = new TalonFX(IDConstants.climberWinchMotorID);
    m_ClimberWinch.getConfigurator().apply(config);
    m_ClimberWinch.setPosition(Constants.ClimberConstants.stowWinchPos);
    
    manualScale = Constants.ClimberConstants.manualScale;
    
    motionMagic = new MotionMagicVoltage(0);

    setClimberStatus(ClimberStatus.STOW);
  }

  public double getClimberPos()
    {return m_ClimberWinch.getPosition().getValueAsDouble();}
  
  public ClimberStatus getClimberStatus()
    {return status;}
  
  public void setClimberStatus(ClimberStatus newStatus)
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

  public boolean isUnlocked()
    {return climbActive;}

  public boolean manualOveride(double motorSpeed)
  {
    speed = motorSpeed;
    setClimberStatus(ClimberStatus.MANUAL);
    return true;
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Climber Position", m_ClimberWinch.getPosition().getValueAsDouble());
    if (RobotContainer.s_Diffector.getRelativeTarget().equals(Constants.DiffectorConstants.coralIntakePosition) || RobotContainer.algae)
    {
      if (!climberClearanceFlag)
      {
        setClimberStatus(ClimberStatus.INTAKE);
        climberClearanceFlag = true;
      }
    }
    else if (climberClearanceFlag)
    {
      setClimberStatus(ClimberStatus.STOW);
      climberClearanceFlag = false;
    }

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
        m_ClimberWinch.setControl(motionMagic.withPosition(Constants.ClimberConstants.climbWinchPos));
        SmartDashboard.putNumber("Climber Target", Constants.ClimberConstants.climbWinchPos);
        // TODO: Merge in active balancing, only in override mode
        break;
      
      case INTAKE:
        m_ClimberWinch.setControl(motionMagic.withPosition(Constants.ClimberConstants.intakeWinchPos));
        SmartDashboard.putNumber("Climber Target", Constants.ClimberConstants.intakeWinchPos);
        break;

      case MANUAL:
        if (speed != 0)
          {m_ClimberWinch.set(speed * manualScale);}
        else
          {m_ClimberWinch.setControl(motionMagic.withPosition(getClimberPos()));}
        break;
    }
  }
}
