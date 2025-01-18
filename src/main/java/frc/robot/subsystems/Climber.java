package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class Climber extends SubsystemBase {
  private final MotionMagicVoltage motionMagic;
  private TalonFX m_Claws;
  private TalonFX m_Winch;
  
  double clawTarget;
  double winchTarget;
  private double[] climbTargets = new double[2];
  
  
  public enum ClimberStatus 
  {
    INIT_CONFIG,
    DEPLOY_CONFIG,
    CLIMB_CONFIG
  };
  
  /** Creates a new Climber. */
  public Climber() 
  { 
    m_Claws = new TalonFX(Constants.Climber.MotorID.m_Claws);
    m_Winch = new TalonFX(Constants.Climber.MotorID.m_Winch);
    motionMagic = new MotionMagicVoltage(0);
    clawTarget = 0;
    winchTarget = 0;
  }

private void setClimberSpeed (double speed)
  {
    m_Claws.set(speed);
    m_Winch.set(speed);
  };

private void setClawTargetPos (double newClawTarget)
{
  clawTarget = newClawTarget;
}

private void setWinchTargetPos (double newWinchTarget)
{
  winchTarget = newWinchTarget;
}

private void setClimbTargets(double newClawTarget, double newWinchTarget)
{
  climbTargets[0] = newClawTarget;
  climbTargets[1] = newWinchTarget;
}

public void setClimberStatus (ClimberStatus Status)
{
    switch (Status)
    {
        case INIT_CONFIG:
        setClimberSpeed(Constants.Climber.MotorSpeeds.m_InitSpeed);
        setClimbTargets(Constants.Climber.ClimberPos.m_InitClawPos, Constants.Climber.ClimberPos.m_InitWinchPos);
        break;

        case DEPLOY_CONFIG:
        setClimberSpeed(Constants.Climber.MotorSpeeds.m_DeploySpeed);
        setClimbTargets(Constants.Climber.ClimberPos.m_DeployClawPos, Constants.Climber.ClimberPos.m_DeployWinchPos);
        break;

        case CLIMB_CONFIG:
        setClimberSpeed(Constants.Climber.MotorSpeeds.m_ClimbSpeed);
        setClimbTargets(Constants.Climber.ClimberPos.m_ClimbClawPos, Constants.Climber.ClimberPos.m_ClimbWinchPos);
        break;
    }
}

  @Override
  public void periodic()
  {
    m_Claws.setControl(motionMagic.withPosition(climbTargets[0]));
    m_Winch.setControl(motionMagic.withPosition(climbTargets[1]));
  }

}
