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
import frc.robot.util.Conversions;

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
  
  /* Name is effect of motor when running anticlockwise/positive (e.g. elevator Up, arm Anticlockwise) */
  /** starboard-side motor(?), forward direction drives carriage up and anticlockwise */
  private static TalonFX m_diffectorUA;
  /** port-side motor(?), forward direction drives carriage down and anticlockwise */
  private static TalonFX m_diffectorDA;
  private DutyCycleEncoder encoder;
  
  private double[] motorTargets = new double[2];
  // Virtual "motors" for simulation; TODO: replace all instances before actual use
  private double simUA = 0;
  private double simDA = 0;

  private double targetElevation;
  private double targetAngle;

  private double offset;
  private double reverseOffset;
  private double armPos;
  private double elevatorPos;
  private static ArmCalculator arm;
  public static boolean stowRequested = true;

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

    targetElevation = Constants.Diffector.ArmPresets.startElevation;
    targetAngle = Constants.Diffector.ArmPresets.startAngle;

    m_diffectorUA.getConfigurator().apply(motorConfig);
    m_diffectorDA.getConfigurator().apply(motorConfig);

    m_diffectorUA.setPosition((Constants.Diffector.ArmPresets.startAngle / rotationRatio) + (Constants.Diffector.ArmPresets.startElevation / travelRatio));
    m_diffectorDA.setPosition((Constants.Diffector.ArmPresets.startAngle / rotationRatio) - (Constants.Diffector.ArmPresets.startElevation / travelRatio));
    simUA = ((Constants.Diffector.ArmPresets.startAngle / rotationRatio) + (Constants.Diffector.ArmPresets.startElevation / travelRatio));
    simDA = ((Constants.Diffector.ArmPresets.startAngle / rotationRatio) - (Constants.Diffector.ArmPresets.startElevation / travelRatio));

    cargoState = updateCargoState();

    motionMagicRequester = new MotionMagicVoltage(0);
  }

  /**
   * Calculates elevator height based on motor positions
   * @return Elevator height in m
   */
  public double getElevatorPos()
  {
    return ((simUA - simDA) * travelRatio) / 2;
    //return ((m_diffectorUA.getPosition().getValueAsDouble() - m_diffectorDA.getPosition().getValueAsDouble()) * travelRatio);
  }

  /**
   * Calculates arm rotation based on motor positions
   * @return Arm rotation, degrees anticlockwise, 0 = algae at top
   */
  public double getArmPos()
  {
    return ((simUA + simDA) * rotationRatio);
    //return (m_diffectorUA.getPosition().getValueAsDouble() + m_diffectorDA.getPosition().getValueAsDouble()) * rotationRatio;
  }

  /**
   * Arm Rotation as measured from encoder
   * @return Arm rotation, degrees anticlockwise, 0 = coral at top [0..360]
   */
  public double getEncoderPos()
  {
    // Encoder outputs [0..1] and is geared 1:1 to the arm
    return (1 - encoder.get()) * 360;
  }

  /**
   * Calculates the position to drive each motor to, based on the target positions for the elevator and arm
   * @param elevatorTarget Target height of the elevator carriage, metres above the ground
   * @param armTarget Target angle of the arm, degrees anticlockwise, 0 = unwound with coral at top
   * @return [motor1 target, motor2 target]
   */
  public double[] calculateMotorTargets(double elevatorTarget, double armTarget)
  {
    // IK projection and object avoidance
    double[] projectedTargets = arm.pathfindArm(elevatorTarget, armTarget, elevatorPos, armPos);
    SmartDashboard.putNumberArray("Arm Path:", projectedTargets);

    double[] calculatedTargets = new double[2];

    calculatedTargets[0] = (projectedTargets[1] / rotationRatio) + (projectedTargets[0] / travelRatio);
    calculatedTargets[1] = (projectedTargets[1] / rotationRatio) - (projectedTargets[0] / travelRatio);

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
    {return (targetElevation == Constants.Diffector.ArmPresets.climbElevation && elevatorAtElevation());}

  /** 
   * Sets the Diffector arm to unwind to starting position 
   * @return Safe to stow
   */
  public boolean unwind()
  {
    targetAngle = Constants.Diffector.ArmPresets.startAngle;
    return (stowRequested = Math.abs(armPos) < stowThreshold);
  }

  /**
   * Sets the Diffector arm to rotate the shortest path to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goShortest(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(armPos, 360), -180, 180);

    if (armPos + offset > maxAbsPos)
      {targetAngle = (armPos + offset - 360);}

    else if (armPos + offset < -maxAbsPos)
      {targetAngle = (armPos + offset + 360);}

    else
      {targetAngle = (armPos + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Clockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goClockwise(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(armPos, 360), -360, 0);

    if (armPos + offset > maxAbsPos)
      {targetAngle = (armPos + offset - 360);}

    else if (armPos + offset < -maxAbsPos)
      {targetAngle = (armPos + offset + 360);}

    else
      {targetAngle = (armPos + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Anticlockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goAnticlockwise(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(armPos, 360), 0, 360);

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
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goToAngle(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(armPos, 360), -180, 180);

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

  @Override
  public void periodic() 
  { 
    //cargoState = updateCargoState(); // TODO: Don't need to call this in periodic, only needs to be called when state changes
 
    calculateMotorTargets();

    simUA += Conversions.clamp(motorTargets[0] - simUA, -100, 100);
    simDA += Conversions.clamp(motorTargets[1] - simDA, -100, 100);

    SmartDashboard.putNumberArray("MotorTargets", motorTargets);
    SmartDashboard.putNumberArray("MotorActual", new double[] {simUA,simDA});
    //m_diffectorUA.setControl(motionMagicRequester.withPosition(motorTargets[0]).withSlot(getSlot()));
    //m_diffectorDA.setControl(motionMagicRequester.withPosition(motorTargets[1]).withSlot(getSlot()));

    armPos = getArmPos();
    elevatorPos = getElevatorPos();
    
    if (stowRequested && (Math.abs(armPos) > stowThreshold || Math.abs(targetAngle) > stowThreshold))
     {stowRequested = false;}

    SmartDashboard.putNumber("Elevator Target", targetElevation);
    SmartDashboard.putNumber("Arm Target", targetAngle);
    SmartDashboard.putNumber("Elevator Height", elevatorPos);
    SmartDashboard.putNumber("Arm Rotation", armPos);
    SmartDashboard.putBoolean("Existance", true);

    //SmartDashboard.putNumber("Angle Test", simTest);
    //SmartDashboard.putNumber("Min Height", arm.checkAngle(simTest));
    //simTest -= 0.1;
  }
}
