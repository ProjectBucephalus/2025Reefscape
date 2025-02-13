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
    m_Winch = new TalonFX(Constants.ClimberConstants.winchID);

    m_Winch.getConfigurator().apply(CTREConfigs.climberWinchFXConfiguration);

    motionMagic = new MotionMagicVoltage(0);
  }

  public void setClimberSpeed(double speed)
    {m_Winch.set(speed);}

  public void setClimbTargets(double newWinchTarget)
  {
    climbTarget = newWinchTarget;
    //Use for auto-positoning
  }

  public double getClimbTargets()
    {return climbTarget;}

  public void setClimberStatus(ClimberStatus Status)
  {
    switch (Status)
    {
      case INIT_CONFIG:
        setClimberSpeed(Constants.ClimberConstants.initSpeed);
        setClimbTargets(Constants.ClimberConstants.initWinchPos);
        break;

      case DEPLOY_CONFIG: 
        if (RobotContainer.s_Intake.isCoralStowed() && RobotContainer.s_Diffector.safeToMoveClimber())
        {
          setClimberSpeed(Constants.ClimberConstants.deploySpeed);
          setClimbTargets(Constants.ClimberConstants.deployWinchPos);
        }   
        break;

      case CLIMB_CONFIG:
        if (RobotContainer.s_Intake.isCoralStowed() && RobotContainer.s_Diffector.safeToMoveClimber())
        {
          setClimberSpeed(Constants.ClimberConstants.climbSpeed);
          setClimbTargets(Constants.ClimberConstants.climbWinchPos);
        }
        break;
    }
  }

  public boolean isStowed()
    {return (m_Winch.getPosition()).getValueAsDouble() <= Constants.ClimberConstants.initWinchThreshold;}

  public boolean climbReady()
    {return (m_Winch.getPosition()).getValueAsDouble() >= Constants.ClimberConstants.deployWinchPos;}

  @Override
  public void periodic()
    {m_Winch.setControl(motionMagic.withPosition(climbTarget));}
}
