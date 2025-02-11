package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CTREConfigs;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {
  private final MotionMagicVoltage motionMagic;
  private TalonFX m_Winch;
  private double climbTarget;
  
  public enum ClimberStatus 
  {
    INIT_CONFIG,
    DEPLOY_CONFIG,
    CLIMB_CONFIG
  };
  
  /** Creates a new Climber. */
  public Climber() 
  { 
    m_Winch = new TalonFX(Constants.Climber.winchID);

    m_Winch.getConfigurator().apply(CTREConfigs.climberWinchFXConfiguration);

    motionMagic = new MotionMagicVoltage(0);
  }

  private void setClimberSpeed(double speed)
    {m_Winch.set(speed);}

  private void setClimbTargets(double newWinchTarget)
  {
    climbTarget = newWinchTarget;
    //Use for auto-positoning
  }

  public void setClimberStatus(ClimberStatus Status)
  {
    switch (Status)
    {
      case INIT_CONFIG:
        setClimberSpeed(Constants.Climber.initSpeed);
        setClimbTargets(Constants.Climber.initWinchPos);
        break;

      case DEPLOY_CONFIG: 
        if (RobotContainer.s_Intake.isCoralStowed() && RobotContainer.s_Diffector.safeToMoveClimber())
        {
          setClimberSpeed(Constants.Climber.deploySpeed);
          setClimbTargets(Constants.Climber.deployWinchPos);
        }   
        break;

      case CLIMB_CONFIG:
        if (RobotContainer.s_Intake.isCoralStowed() && RobotContainer.s_Diffector.safeToMoveClimber())
        {
          setClimberSpeed(Constants.Climber.climbSpeed);
          setClimbTargets(Constants.Climber.climbWinchPos);
        }
        break;
    }
  }

  public boolean isStowed()
    {return (m_Winch.getPosition()).getValueAsDouble() <= Constants.Climber.initWinchThreshold;}

  public boolean climbReady()
    {return (m_Winch.getPosition()).getValueAsDouble() >= Constants.Climber.deployWinchPos;}

  @Override
  public void periodic()
    {m_Winch.setControl(motionMagic.withPosition(climbTarget));}
}
