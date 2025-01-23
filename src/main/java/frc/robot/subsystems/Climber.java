package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class Climber extends SubsystemBase {
  private final MotionMagicVoltage motionMagic;
  private TalonFX m_Claws;
  private TalonFX m_Winch;
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
  }

  private void setClimberSpeed(double speed)
  {
    m_Claws.set(speed);
    m_Winch.set(speed);
  };

private void setClawTargetPos (double newClawTarget)
{
  climbTargets[0] = newClawTarget;
  //Use for Manual Controls as to not set winch as well
}

private void setWinchTargetPos (double newWinchTarget)
{
  climbTargets[1] = newWinchTarget;
  //Use for Manual Controls as to not set claws as well
}

private void setClimbTargets(double newClawTarget, double newWinchTarget)
{
  climbTargets[0] = newClawTarget;
  climbTargets[1] = newWinchTarget;
  //Use for auto-positoning
}

  public void setClimberStatus(ClimberStatus Status)
  {
    switch (Status)
    {
      case INIT_CONFIG:
      setClimberSpeed(Constants.Climber.MotorSpeeds.initSpeed);
      setClimbTargets(Constants.Climber.ClimberPos.initClawPos, Constants.Climber.ClimberPos.initWinchPos);
      break;

      case DEPLOY_CONFIG:
      setClimberSpeed(Constants.Climber.MotorSpeeds.deploySpeed);
      setClimbTargets(Constants.Climber.ClimberPos.deployClawPos, Constants.Climber.ClimberPos.deployWinchPos);
      break;

      case CLIMB_CONFIG:
      setClimberSpeed(Constants.Climber.MotorSpeeds.climbSpeed);
      setClimbTargets(Constants.Climber.ClimberPos.climbClawPos, Constants.Climber.ClimberPos.climbWinchPos);
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
