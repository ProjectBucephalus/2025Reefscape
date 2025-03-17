package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.IDConstants;
import frc.robot.util.SD;

public class Climber extends SubsystemBase
{
  /* Declarations of the motor controller */
  private TalonFX winchMotor;
  private ClimberStatus status;

  /* Declarations of all the motion magic variables */
  private final MotionMagicVoltage motionMagic;
  private double speed;
  private TalonFXConfiguration motorConfig = new TalonFXConfiguration()
  {{
    /* Climber Values */
    Feedback.SensorToMechanismRatio = Constants.ClimberConstants.winchGearRatio;
    MotionMagic.MotionMagicCruiseVelocity = Constants.ClimberConstants.winchDefaultCruise;
    MotionMagic.MotionMagicAcceleration = Constants.ClimberConstants.winchMotionMagicAccel;
    Slot0.kP = Constants.ClimberConstants.winchKP;
    Slot0.kI = Constants.ClimberConstants.winchKI;
    Slot0.kD = Constants.ClimberConstants.winchKD;
    MotorOutput.NeutralMode = NeutralModeValue.Brake;
    MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }};

  public enum ClimberStatus 
  {
    ACTIVE,
    STOW,
    MANUAL,
    CLIMB
  };

  public Climber() 
  { 
    winchMotor = new TalonFX(IDConstants.climberWinchMotorID);
    winchMotor.getConfigurator().apply(motorConfig);
    winchMotor.setPosition(ClimberConstants.stowWinchPos);
    
    motionMagic = new MotionMagicVoltage(0);

    setStatus(ClimberStatus.STOW);
  }
  
  private void setStatus(ClimberStatus newStatus)
  {
    
    if (newStatus == ClimberStatus.CLIMB)
    {
      if (status == ClimberStatus.STOW)
      { // Active state has required protections for leaving Stow position
        status = ClimberStatus.ACTIVE;
      }
      else
      {
        winchMotor.getConfigurator().apply(motorConfig.MotionMagic.withMotionMagicCruiseVelocity(ClimberConstants.winchClimbCruise));
        status = newStatus;
      }
    }
    else
    {
      winchMotor.getConfigurator().apply(motorConfig.MotionMagic.withMotionMagicCruiseVelocity(ClimberConstants.winchDefaultCruise));
      status = newStatus;
    }
  }

  public Command setStatusCommand(ClimberStatus status)
  {
    return Commands.runOnce(() -> this.setStatus(status), this);
  }

  public boolean isUnlocked()
    {return status != ClimberStatus.STOW;}

  public boolean manualOveride(double motorSpeed)
  {
    speed = motorSpeed;
    setStatus(ClimberStatus.MANUAL);
    return true;
  }

  @Override
  public void periodic()
  {
    SD.CLIMBER_POS.put(winchMotor.getPosition().getValueAsDouble());

    switch (status)
    {
      case STOW:
        winchMotor.setControl(motionMagic.withPosition(ClimberConstants.stowWinchPos));
        SD.CLIMBER_TARGET.put(ClimberConstants.stowWinchPos);
        break;

      case ACTIVE:
        if (RobotContainer.diffector.climbSafe())
        {
          winchMotor.setControl(motionMagic.withPosition(ClimberConstants.activeWinchPos));
          SD.CLIMBER_TARGET.put(ClimberConstants.activeWinchPos);
        }
        else
        {
          winchMotor.setControl(motionMagic.withPosition(winchMotor.getPosition().getValueAsDouble()));
        }
        break;

      case CLIMB:
        if (RobotContainer.diffector.climbReady())
        {
          double adjustedClimberPos = ClimberConstants.climbWinchPos;
          
          if (SD.OVERRIDE.get()) 
          {
            adjustedClimberPos += RobotContainer.swerve.getPigeon2().getPitch().getValueAsDouble() * ClimberConstants.winchBalanceScalar;
            adjustedClimberPos = MathUtil.clamp(adjustedClimberPos, ClimberConstants.climbActiveInnerLimit, ClimberConstants.climbActiveOuterLimit);
          }

          winchMotor.setControl(motionMagic.withPosition(adjustedClimberPos));
          SD.CLIMBER_TARGET.put(adjustedClimberPos);
        }
        else
        {
          winchMotor.setControl(motionMagic.withPosition(winchMotor.getPosition().getValueAsDouble()));
        }
      
        break;

      case MANUAL:
        if (speed != 0)
          {winchMotor.set(speed * ClimberConstants.manualScale);}
        else
          {winchMotor.setControl(motionMagic.withPosition(winchMotor.getPosition().getValueAsDouble()));}
        break;
    }
  }
}
