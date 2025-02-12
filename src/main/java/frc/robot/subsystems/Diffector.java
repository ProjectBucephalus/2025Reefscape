// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;
import frc.robot.util.ArmCalculator;

public class Diffector extends SubsystemBase 
{
  public enum CargoStates{EMPTY, ONE_ITEM, TWO_ITEM}
  private CargoStates cargoState;

  private final MotionMagicVoltage motionMagicRequester;
  private final double rotationRatio;
  private final double travelRatio;
  private final TalonFXConfiguration motorConfig;
  private final double maxAbsPos = Constants.Diffector.maxAbsPos;
  private final double turnBackThreshold = Constants.Diffector.turnBackThreshold;
  private final double stowThreshold = Constants.Diffector.angleTolerance;
  
  /* Name is effect of motor when running clockwise/positive (e.g. elevator Up, arm Clockwise) */
  /** starboardside motor(?), forward direction drives carriage up and anticlockwise */
  private static TalonFX m_diffectorUA;
  /** portside motor(?), forward direction drives carriage down and anticlockwise */
  private static TalonFX m_diffectorDA;
  private DutyCycleEncoder encoder;

  private double[] motorTargets = new double[2];

  private double targetElevation;
  private double targetAngle;

  private double offset;
  private double reverseOffset;
  public double armPos;
  public double elevatorPos;
  private static ArmCalculator arm;

  /** Creates a new Diffector. */
  public Diffector() 
  {
    arm = new ArmCalculator();
    motorConfig = CTREConfigs.diffectorFXConfig;
    rotationRatio = Constants.Diffector.rotationRatio;
    travelRatio = Constants.Diffector.travelRatio;

    m_diffectorUA = new TalonFX(Constants.Diffector.uaMotorID);
    m_diffectorDA = new TalonFX(Constants.Diffector.daMotorID);
    encoder = new DutyCycleEncoder(Constants.Diffector.encoderPWMID);

    targetElevation = Constants.Diffector.startElevation;
    targetAngle = 0;

    m_diffectorUA.getConfigurator().apply(motorConfig);
    m_diffectorDA.getConfigurator().apply(motorConfig);

    m_diffectorUA.setPosition((Constants.Diffector.startAngle / rotationRatio) + (Constants.Diffector.startElevation / travelRatio));
    m_diffectorDA.setPosition((Constants.Diffector.startAngle / rotationRatio) - (Constants.Diffector.startElevation / travelRatio));

    cargoState = updateCargoState();
    updateLocalPos();

    motionMagicRequester = new MotionMagicVoltage(0);
  }

  /**
   * Calculates arm rotation based on motor positions
   * @return Arm rotation, degrees clockwise, 0 = algae at top
   */
  public double getArmPos()
    {return ((m_diffectorUA.getPosition().getValueAsDouble() + m_diffectorDA.getPosition().getValueAsDouble()) * rotationRatio) / 2;}
    /**
   * Arm Rotation as measured from encoder
   * @return Arm rotation, degrees clockwise, 0 = coral at top
   */
  public double getEncoderPos()
  {
    // Encoder outputs [0..1] and is geared 1:1 to the arm
    return (1 - encoder.get()) * 360;
  }

  /**
   * Calculates elevator height based on motor positions
   * @return Elevator height in m
   */
  public double getElevatorPos()
    {return ((m_diffectorUA.getPosition().getValueAsDouble() - m_diffectorDA.getPosition().getValueAsDouble())/2) * travelRatio;}

  /**
   * Calculates the position to drive each motor to, based on the target positions for the elevator and arm
   * @param elevatorTarget Target height of the elevator carriage, metres above the ground
   * @param armTarget Target angle of the arm, degrees anticlockwise, 0 = unwound with coral at top
   * @return [motor1 target, motor2 target]
   */
  public double[] calculateMotorTargets(double elevatorTarget, double armTarget)
  {
    double[] calculatedTargets = new double[2];
    
    if (arm.checkAngle(armTarget) > elevatorPos)
    {
      armTarget = armPos;
    }

    if (elevatorTarget < elevatorPos && arm.checkAngle(armPos) > elevatorTarget) 
    {
      elevatorTarget = elevatorPos;
    }

    calculatedTargets[0] = (armTarget / rotationRatio) + (elevatorTarget / travelRatio);
    calculatedTargets[1] = (armTarget / rotationRatio) - (elevatorTarget / travelRatio);

    return calculatedTargets;
  }

  private void calculateMotorTargets()
    {motorTargets = calculateMotorTargets(targetElevation, targetAngle);}

  /** Returns true if the diffector is at its current target angle */
  public boolean armAtAngle()
    {return Math.abs(armPos - targetAngle) < Constants.Diffector.angleTolerance;}

  /** Returns true if the diffector is at its current target elevation */
  public boolean elevatorAtElevation()
    {return Math.abs(elevatorPos - targetElevation) < Constants.Diffector.elevationTolerance;}

  /** Returns true if the diffector is safely in climb position */
  public boolean climbReady()
    {return (targetElevation == Constants.Diffector.climbElevation && elevatorAtElevation());}

  /** 
   * Sets the Diffector arm to unwind to starting position 
   * @return Safe to stow
   */
  public boolean unwind()
  {
    targetAngle = Constants.Diffector.startAngle;
    return (Math.abs(armPos) < stowThreshold);
  }

  /**
   * Sets the Diffector arm to rotate the shortest path to the target angle, with protection against over-rotation
   * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goShortest(double targetAngle)
  {
    targetAngle %= 360;
    offset = MathUtil.inputModulus(targetAngle - (armPos % 360), -180, 180);

    if (armPos + offset > maxAbsPos)
      {targetAngle = (armPos + offset - 360);}

    else if (armPos + offset < -maxAbsPos)
      {targetAngle = (armPos + offset + 360);}

    else
      {targetAngle = (armPos + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Clockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goClockwise(double targetAngle)
  {
    targetAngle %= 360;
    offset = MathUtil.inputModulus(targetAngle - (armPos % 360), -360, 0);

    if (armPos + offset > maxAbsPos)
      {targetAngle = (armPos + offset - 360);}

    else if (armPos + offset < -maxAbsPos)
      {targetAngle = (armPos + offset + 360);}

    else
      {targetAngle = (armPos + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Anticlockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goAnticlockwise(double targetAngle)
  {
    targetAngle %= 360;
    offset = MathUtil.inputModulus(targetAngle - (armPos % 360), 0, 360);

    if (armPos + offset > maxAbsPos)
      {targetAngle = (armPos + offset - 360);}

    else if (armPos + offset < -maxAbsPos)
      {targetAngle = (armPos + offset + 360);}

    else
      {targetAngle = (armPos + offset);}
  }

  /**
   * Sets the Diffector arm to rotate the safest path to the target angle, with protection against over-rotation. 
   * Below a threshold will go shortest path, otherwise will minimise total rotations
   * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goToAngle(double targetAngle)
  {
    targetAngle %= 360;
    offset = MathUtil.inputModulus(targetAngle - (armPos % 360), -180, 180);

    if (Math.abs(offset) >= turnBackThreshold)
    {
      reverseOffset = offset - Math.copySign(360, offset);

      if (Math.abs(armPos + offset) > Math.abs(armPos + reverseOffset))
        {targetAngle = (armPos + reverseOffset);}
      
      else 
        {targetAngle = (armPos + offset);}
    }
    else if (armPos + offset > maxAbsPos)
      {targetAngle = (armPos + offset - 360);}

    else if (armPos + offset < -maxAbsPos)
      {targetAngle = (armPos + offset + 360);}

    else
      {targetAngle = (armPos + offset);}
  }

  public double getArmTarget()
    {return targetAngle;}

  public void setElevatorTarget(double newTarget)
    {targetElevation = newTarget;}

  public double getElevatorTarget()
    {return targetElevation;}

  public boolean safeToMoveCoral()
  {
    return Constants.Diffector.coralElevatorLowTheshold < elevatorPos 
    && elevatorPos < Constants.Diffector.coralElevatorHighThreshold;
  }

  public boolean safeToMoveAlgae()
  {
    return Constants.Diffector.algaeElevatorLowTheshold < elevatorPos 
    && elevatorPos < Constants.Diffector.algaeElevatorHighThreshold;
  }

  public boolean safeToMoveClimber()
  {
    return Constants.Diffector.climberElevatorLowTheshold < elevatorPos 
    && elevatorPos < Constants.Diffector.climberElevatorHighThreshold;
  }

  /** Returns the ID of the motor control slot to use */
  private int getSlot()
  {
    switch (cargoState) 
    {
      case EMPTY: return 0;
      case ONE_ITEM: return 1;
      case TWO_ITEM: return 2;
      default: return 0;
    }
  }

  private CargoStates updateCargoState()
  {
    if(RobotContainer.coral && RobotContainer.algae) // Both game pieces
      {return CargoStates.TWO_ITEM;}
    else if(RobotContainer.coral ^ RobotContainer.algae) // One game piece
      {return CargoStates.ONE_ITEM;}
    else if(!RobotContainer.coral && !RobotContainer.algae) // No game piece
      {return CargoStates.EMPTY;}
    else // Default state, should never be reached
      {return CargoStates.EMPTY;}
  }

  public void testingOveride(boolean a, double input)
  {
    if (a)
    {
      m_diffectorUA.set(input);
    }
    else
    {
      m_diffectorDA.set(input);
    }
  }

  private void updateLocalPos()
  {
    armPos = getArmPos();
    elevatorPos = getElevatorPos();
  }

  @Override
  public void periodic() 
  { 
    updateLocalPos();

    cargoState = updateCargoState(); // TODO: Don't need to call this in periodic, only needs to be called when state changes

    calculateMotorTargets();

    m_diffectorUA.setControl(motionMagicRequester.withPosition(motorTargets[0]).withSlot(getSlot()));
    m_diffectorDA.setControl(motionMagicRequester.withPosition(motorTargets[1]).withSlot(getSlot()));

    SmartDashboard.putNumber("Elevator Height", elevatorPos);
    SmartDashboard.putNumber("Arm Rotation", armPos);

    m_diffectorUA.setPosition(m_diffectorUA.getPosition().getValueAsDouble() + 10);
    m_diffectorDA.setPosition(m_diffectorDA.getPosition().getValueAsDouble() + 10);
  }
}
