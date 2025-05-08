package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.IDConstants;
import frc.robot.constants.MechanismConstants.ClimberConfigs;
import frc.robot.util.Conversions;
import frc.robot.util.SD;

public class Climber extends SubsystemBase
{
  /* Declarations of the motor controller */
  private TalonFX m_Climber;
  private Status status;

  /* Declarations of all the motion magic variables */
  private final MotionMagicVoltage motionMagic;
  private double speed;
  private TalonFXConfiguration motorConfig = ClimberConfigs.climberMotorConfig;
  private boolean climbLocked = true;

  public enum Status 
  {
    ACTIVE,
    STOW,
    MANUAL,
    CLIMB,
    HOLD
  };

  public Climber() 
  { 
    m_Climber = new TalonFX(IDConstants.climberWinchMotorID);
    m_Climber.getConfigurator().apply(motorConfig);
    m_Climber.setPosition(ClimberConstants.startWinchPos);
    
    motionMagic = new MotionMagicVoltage(0);

    setStatus(Status.STOW);
  }
  
  private void setStatus(Status newStatus)
  {
    // Go from stow to active before climb
    if (newStatus == Status.CLIMB && status == Status.STOW)
      {status = Status.ACTIVE;}
    // Go from climb to stow before active
    else if (newStatus == Status.ACTIVE && status == Status.CLIMB) 
      {status = Status.STOW;}
    else 
      {status = newStatus;}
  }

  public Command setStatusCommand(Status status)
    {return runOnce(() -> setStatus(status)).withName("SetClimberStatus");}

  public double getPos()
    {return m_Climber.getPosition().getValueAsDouble();}

  public boolean climbReady()
    {return getPos() >= ClimberConstants.prepareWinchPos;}

  public boolean armSafe()
    {return getPos() >= ClimberConstants.safeWinchPos;}  

  public boolean driverRumbleAngle()
    {return getPos() < ClimberConstants.startDrivePos;}

  public boolean offGround()
    {return getPos() < ClimberConstants.offGroundPos;}  

  public boolean manualOveride(double motorSpeed)
  {
    speed = motorSpeed;
    setStatus(Status.MANUAL);
    return true;
  }

  public void unlockClimb()
    {climbLocked = false;}

  @Override
  public void periodic()
  {
    SD.CLIMBER_POS.put(getPos());
    if (SD.CLIMB_OVERRIDE.get()) {unlockClimb();}

    switch (status)
    {
      case STOW:
        m_Climber.setControl(motionMagic.withPosition(ClimberConstants.startWinchPos));
        SD.CLIMBER_TARGET.put(ClimberConstants.startWinchPos);
        break;

      case ACTIVE:
        m_Climber.setControl(motionMagic.withPosition(ClimberConstants.prepareWinchPos));
        SD.CLIMBER_TARGET.put(ClimberConstants.prepareWinchPos);
        break;

      case CLIMB:
        double adjustedClimberPos = getPos();
        
        if (SD.OVERRIDE.get() && (adjustedClimberPos < ClimberConstants.offGroundPos)) 
        {
          adjustedClimberPos += Units.degreesToRotations((RobotContainer.s_Swerve.getPigeon2().getPitch().getValueAsDouble() - ClimberConstants.targetRobotClimbPitch) * ClimberConfigs.winchBalanceScalar);
          adjustedClimberPos = Conversions.clamp(adjustedClimberPos, ClimberConstants.climbActiveInnerLimit, ClimberConstants.climbActiveOuterLimit);
        }
        else
        {
          adjustedClimberPos = ClimberConstants.climbWinchPos;
        }

        m_Climber.setControl(motionMagic.withPosition(adjustedClimberPos));
        SD.CLIMBER_TARGET.put(adjustedClimberPos);
        break;

      case MANUAL:
        if 
        (
          (
            (speed > 0 && getPos() <= 1.05 * ClimberConstants.prepareWinchPos) || 
            (speed < 0 && getPos() >= 0) || 
            (speed != 0 && SD.CLIMB_OVERRIDE.get())
          ) 
          && !climbLocked
        )
          {m_Climber.set(speed * Control.manualClimberScale);}
        else
        {
          m_Climber.setControl(motionMagic.withPosition(getPos()));
          setStatus(Status.HOLD);
        }
        break;
      
      case HOLD:
        break;
    }
    SmartDashboard.putString("Climber State", status.name());
  }
}
