package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
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

  public enum ClimberStatus 
  {
    ACTIVE,
    LOCKED,
    MANUAL,
    CLIMB
  };

  public Climber() 
  { 
    this.status = ClimberStatus.LOCKED;

    m_ClimberWinch = new TalonFX(IDConstants.climberWinchMotorID);
    m_ClimberWinch.getConfigurator().apply(CTREConfigs.climberWinchFXConfig);
    m_ClimberWinch.setPosition(Constants.ClimberConstants.lockedWinchPos / 360);
    
    manualScale = 0.25;

    motionMagic = new MotionMagicVoltage(0);
  }

  public double getClimberPos()
    {return m_ClimberWinch.getPosition().getValueAsDouble();}
  
  public ClimberStatus getClimberStatus()
    {return status;}
  
  public void setClimberStatus(ClimberStatus newStatus)
    {status = newStatus;}

  public boolean isUnlocked()
    {return climbActive;}

  public boolean manualOveride(double motorSpeed)
  {
    speed = motorSpeed;
    if (status == ClimberStatus.LOCKED)
      {return false;}

    status = ClimberStatus.MANUAL;
    return true;
  }

  @Override
  public void periodic()
  {
    switch (status)
    {
      case LOCKED:
        m_ClimberWinch.setControl(motionMagic.withPosition(Constants.ClimberConstants.lockedWinchPos / 360));
        break;

      case ACTIVE:
        m_ClimberWinch.setControl(motionMagic.withPosition(Constants.ClimberConstants.activeWinchPos / 360));
        break;

      case CLIMB:
        m_ClimberWinch.setControl(motionMagic.withPosition(Constants.ClimberConstants.climbWinchPos / 360));
        break;

      case MANUAL:
        if (getClimberPos() > Constants.ClimberConstants.activeWinchPos || getClimberPos() < Constants.ClimberConstants.climbWinchPos)
          {speed = 0;}
        if (speed != 0)
          {m_ClimberWinch.set(speed * manualScale);}
        else
          {m_ClimberWinch.setControl(motionMagic.withPosition(getClimberPos()));}
    }
  }
}
