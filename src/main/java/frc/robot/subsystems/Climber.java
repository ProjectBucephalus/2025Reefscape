package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private TalonFX m_Claws;
  private TalonFX m_Winch;

  public enum ClimberStatus 
  {
    INIT_CONFIG,
    DEPLOY_CONFIG,
    CLIMB_CONFIG
  };

  public Climber() 
  { 
    m_Claws = new TalonFX(Constants.Climber.MotorID.m_Claws);
    m_Winch = new TalonFX(Constants.Climber.MotorID.m_Winch);
  }

private void setClimberSpeed (double speed)
  {
    m_Claws.set(speed);
    m_Winch.set(speed);
  };

private void setClawPos (double pos)
{
    m_Claws.setPosition(pos);
}

private void setWinchPos (double pos)
{
    m_Winch.setPosition(pos);
}

public void setClimberStatus (ClimberStatus Status)
{
    switch (Status)
    {
        case INIT_CONFIG:
        setClimberSpeed(Constants.Climber.MotorSpeeds.m_InitSpeed);
        setClawPos(Constants.Climber.ClimberPos.m_InitClawPos);
        setWinchPos(Constants.Climber.ClimberPos.m_InitWinchPos);
        break;

        case DEPLOY_CONFIG:
        setClimberSpeed(Constants.Climber.MotorSpeeds.m_DeploySpeed);
        setClawPos(Constants.Climber.ClimberPos.m_DeployClawPos);
        setWinchPos(Constants.Climber.ClimberPos.m_DeployWinchPos);
        break;

        case CLIMB_CONFIG:
        setClimberSpeed(Constants.Climber.MotorSpeeds.m_ClimbSpeed);
        setClawPos(Constants.Climber.ClimberPos.m_ClimbClawPos);
        setWinchPos(Constants.Climber.ClimberPos.m_ClimbWinchPos);
        break;
    }
}

  @Override
  public void periodic()
  {

  }

}
