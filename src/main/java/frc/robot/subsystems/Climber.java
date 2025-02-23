package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;
import frc.robot.subsystems.Intake.IntakeStatus;


public class Climber extends SubsystemBase
{
    /* Declarations of all the motor controllers */
  private TalonFX m_ClimberWinch;
  boolean climbActive;
  private ClimberStatus status;

  private DigitalInput dio;

  /* Declarations of all the motion magic variables */
  private final MotionMagicVoltage motionMagic;
  private double climberCurrentPos;
  private double climberTargetPos;

  public enum ClimberStatus 
  {
    ACTIVE,
    LOCKED,
    EXTEND,
    RETRACT,
    CLIMB
  };

  public Climber() 
  { 
    this.status = ClimberStatus.LOCKED;

    m_ClimberWinch = new TalonFX(IDConstants.algaeIntakeRollerID);
    
    climberCurrentPos = 0;

    motionMagic = new MotionMagicVoltage(0);

    dio = new DigitalInput(IDConstants.algaeIntakeDIO); // TODO: Correct Constants ID + Is it actually being used?
  }

  public double getClimberPos()
    {return climberCurrentPos;}
  
  public ClimberStatus getClimberStatus()
    {return status;}
  
  public void setClimberStatus(ClimberStatus newStatus)
    {status = newStatus;}

  public boolean isUnlocked()
    {return climbActive;}

  public void setClimberTarget(double newTarget)
    {climberTargetPos = newTarget;}

  @Override
  public void periodic()
  {
    switch (status)
    {
      case LOCKED:
        setClimberTarget(Constants.ClimberConstants.lockedWinchPos);
        break;

      case ACTIVE:
        setClimberTarget(Constants.ClimberConstants.activeWinchPos);
        break;

      case CLIMB:
        setClimberTarget(Constants.ClimberConstants.climbWinchPos);
        break;

      case RETRACT:
        //TODO
      case EXTEND:
        //TODO
    }
  }
}
