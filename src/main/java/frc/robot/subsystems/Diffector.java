// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DiffectorConstants.IKGeometry;
import frc.robot.constants.IDConstants;
import frc.robot.util.ArmCalculator;
import frc.robot.util.ArmPathPlanner;
import frc.robot.util.Conversions;

public class Diffector extends SubsystemBase 
{
  public enum CargoStates{EMPTY, ONE_ITEM, TWO_ITEM}
  private CargoStates cargoState;

  private final MotionMagicVoltage motionMagicRequester;
  private final double rotationRatio;
  private final double travelRatio;
  private final TalonFXConfiguration motorConfigUA;
  private final TalonFXConfiguration motorConfigDA;
  private final double stowThreshold = Constants.DiffectorConstants.angleTolerance;
  
  /* Name is effect of motor when running anticlockwise/positive (e.g. elevator Up, arm Anticlockwise) */
  /** starboard-side motor(?), forward direction drives carriage up and anticlockwise */
  private static TalonFX m_diffectorUA;
  /** port-side motor(?), forward direction drives carriage down and anticlockwise */
  private static TalonFX m_diffectorDA;
  private CANcoder encoder;

  private double[] motorTargets = new double[2];

  private Translation2d targetPosition;
  private double targetElevation;
  private double targetAngle;
  private Translation2d oldTarget;

  private double angle;
  private double elevation;
  private Translation2d armPosition;

  public static ArmCalculator arm;
  public static boolean transferRequested = false;
  public static boolean stowRequested = true;

  /** Elevation is scaled up and Rotation is scaled down by this factor for the pathplanner map */
  private double projectionElevation = 0.1;
  private double projectionAngle     = 10;
  //private PathConstraints armPathConstraints = new PathConstraints(1, 1, 0, 0);
  //private GoalEndState armEndState = new GoalEndState(0, Rotation2d.kZero);
  private static ArrayList<Translation2d> plannedPathPoints = new ArrayList<Translation2d>();

  /** Creates a new Diffector. */
  public Diffector() 
  {
    arm = new ArmCalculator();
    motorConfigUA = CTREConfigs.diffectorFXConfig;
    motorConfigDA = motorConfigUA;
    motorConfigDA.Slot0.kG = -motorConfigUA.Slot0.kG;
    motorConfigDA.Slot1.kG = -motorConfigUA.Slot1.kG;
    motorConfigDA.Slot2.kG = -motorConfigUA.Slot2.kG;

    rotationRatio = Constants.DiffectorConstants.rotationRatio;
    travelRatio = Constants.DiffectorConstants.travelRatio;

    m_diffectorUA = new TalonFX(IDConstants.uaMotorID);
    m_diffectorDA = new TalonFX(IDConstants.daMotorID);
    encoder = new CANcoder(IDConstants.armCANcoderID);

    targetPosition  = Constants.DiffectorConstants.startPosition;
    targetElevation = targetPosition.getX();
    targetAngle     = targetPosition.getY();

    m_diffectorUA.getConfigurator().apply(motorConfigUA);
    m_diffectorDA.getConfigurator().apply(motorConfigDA);

    m_diffectorUA.setPosition(Units.degreesToRotations((Constants.DiffectorConstants.startPosition.getY() / rotationRatio) + (Constants.DiffectorConstants.startPosition.getX() / travelRatio)));
    m_diffectorDA.setPosition(Units.degreesToRotations((Constants.DiffectorConstants.startPosition.getY() / rotationRatio) - (Constants.DiffectorConstants.startPosition.getX() / travelRatio)));
    
    motorTargets = calculateMotorTargets(targetPosition);

    calculatePosition();
    cargoState = updateCargoState();

    motionMagicRequester = new MotionMagicVoltage(0);

    ArmPathPlanner.ensureInitialized();
    ArmPathPlanner.setStartPosition(ArmPathPlanner.fromArmRelative(armPosition));
    ArmPathPlanner.setGoalPosition(ArmPathPlanner.fromArmRelative(targetPosition, false));
    plannedPathPoints.clear();
    plannedPathPoints.add(armPosition);
  }

  /**
   * @return Height of elevator measured from arm-axis to ground, metres
   */
  public double getElevation()
    {return elevation;}

  /**
   * @return Arm rotation, degrees anticlockwise as seen from the front, 0 = coral at top
   */
  public double getAngle()
    {return angle;}
  
  /**
   * Calculates arm elevation and rotation based on motor positions
   * @return Arm position, metres over ground, degrees anticlockwise, 0 = coral at top
   */
  private Translation2d calculatePosition()
  {
    elevation = ((Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()) - Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble())) / 2) * travelRatio;
    angle = ((Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()) + Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble())) * rotationRatio) / 2;
    
    if (atAngle() && atElevation()) 
    {
      if (!MathUtil.isNear(getEncoderPos(), angle, Constants.DiffectorConstants.angleTolerance))
      {
        angle = getEncoderPos();
        m_diffectorUA.setPosition(Units.degreesToRotations((angle / rotationRatio) + (elevation / travelRatio)));
        m_diffectorDA.setPosition(Units.degreesToRotations((angle / rotationRatio) - (elevation / travelRatio)));
        SmartDashboard.putBoolean("encoder overide", true);
      }
      else
        SmartDashboard.putBoolean("encoder overide", false);
    }

    armPosition = new Translation2d(elevation, angle);
    return armPosition;
  }
  
  /**
   * Arm Rotation as measured from encoder
   * @return Arm rotation, degrees anticlockwise, 0 = coral at top
   */
  public double getEncoderPos()
  {
    // Encoder outputs is geared 1:1 to the arm, so output is inverted
    return Units.rotationsToDegrees(encoder.getPosition().getValueAsDouble());
  }

  /**
   * Gets absolute arm rotation
   * @return Arm rotation, wrapped, degrees anticlockwise, 0 = coral at top, [0..360]
   */
  public double getRelativeRotation()
  {
    return Conversions.mod(angle, 360);
  }

  private void calculatePath()
  {
    targetElevation = Math.min(Constants.DiffectorConstants.maxZ, targetElevation);

    targetPosition = new Translation2d(targetElevation, targetAngle);

    if (!targetPosition.equals(oldTarget))
    {
      targetElevation = arm.checkPosition(targetPosition);
      oldTarget = targetPosition;

      plannedPathPoints = arm.pathfindArm(targetPosition, armPosition);
      SmartDashboard.putNumberArray("pathDump", plannedPathPoints.stream().mapMultiToDouble((point, consumer) -> {consumer.accept(point.getX()); consumer.accept(point.getY());}).toArray());
    }

    /*if (ArmPathPlanner.isNewPathAvailable())
    {      
      PathPlannerPath plannedPath = ArmPathPlanner.getCurrentPath(armPathConstraints, armEndState);

      plannedPathPoints.clear();
      if (plannedPath != null)
      { 
        List<Waypoint> plannedPathWaypoints = ArmPathPlanner.getCurrentPath(armPathConstraints, armEndState).getWaypoints();
        
        if (plannedPathWaypoints != null)
          {plannedPathPoints.addAll(plannedPathWaypoints.stream().map(waypoint -> ArmPathPlanner.toArmRelative(waypoint.anchor())).toList());}
        else
          {plannedPathPoints.add(new Translation2d(arm.checkPosition(targetElevation, targetAngle), targetAngle));}
      }
    }*/

    if (plannedPathPoints.size() != 0)
    {
      SmartDashboard.putNumberArray("target Point", new double[]{plannedPathPoints.get(0).getX(), plannedPathPoints.get(0).getY()});
      motorTargets = calculateMotorTargets(plannedPathPoints.get(0));

      if 
      (
        Math.abs(plannedPathPoints.get(0).getX() - elevation) <= projectionElevation &&
        Math.abs(plannedPathPoints.get(0).getY() - angle) <= projectionAngle
      )
        {plannedPathPoints.remove(0);}
    }
  }

  /**
   * Calculates the position to drive each motor to, based on the target positions for the elevator and arm
   * @param elevationTarget Target height of the elevator carriage, metres above the ground
   * @param angleTarget Target angle of the arm, degrees anticlockwise, 0 = unwound with coral at top
   * @return [motor1 target, motor2 target]
   */
  private double[] calculateMotorTargets(Translation2d target)
  {
    double[] calculatedTargets = new double[2];

    calculatedTargets[0] = (target.getY() / rotationRatio) + (target.getX() / travelRatio);
    calculatedTargets[1] = (target.getY() / rotationRatio) - (target.getX() / travelRatio);

    return calculatedTargets;
  }

  /** Returns true if the diffector is at its current target angle */
  public boolean atAngle()
    {return Math.abs(angle - targetAngle) < Constants.DiffectorConstants.angleTolerance;}

  /** Returns true if the diffector is at its current target elevation */
  public boolean atElevation()
    {return Math.abs(elevation - targetElevation) < Constants.DiffectorConstants.elevationTolerance;}

  /** Returns true if the diffector is at its current target elevation and angle */
  public boolean atPosition()
    {return atElevation() && atAngle();}

  /** Returns true if the diffector is safely in climb position */
  public boolean climbReady()
    {return (targetElevation == Constants.DiffectorConstants.climbPosition.getX() && atElevation());}

  /** 
   * Sets the Diffector arm to unwind to starting position 
   * @return Safe to stow
   */
  public boolean unwind()
  {
    targetAngle = Constants.DiffectorConstants.startPosition.getY();
    return (stowRequested = Math.abs(angle) < stowThreshold);
  }

  public double getAngleTarget()
    {return targetAngle;}

  public void setElevationTarget(double newTarget)
    {targetElevation = Conversions.clamp(newTarget, Constants.DiffectorConstants.minZ, Constants.DiffectorConstants.maxZ);}

  public double getElevationTarget()
    {return targetElevation;}

  public boolean safeToMoveClimber()
  {
    return Constants.DiffectorConstants.climberElevatorLowTheshold < elevation 
    && elevation < Constants.DiffectorConstants.climberElevatorHighThreshold;
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
      {m_diffectorUA.set(input);}
    else
      {m_diffectorDA.set(input);}
  }

  public void goToAngle(double newTarget) 
    {targetAngle = arm.goToAngle(newTarget);}

  @Override
  public void periodic() 
  { 
    calculatePosition();
    cargoState = updateCargoState();

    calculatePath();

    m_diffectorUA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[0])).withSlot(0));//getSlot()));
    m_diffectorDA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[1])).withSlot(0));//getSlot()));
    if (transferRequested && MathUtil.isNear(0, Conversions.mod(angle, 360), IKGeometry.latchAngle))
     {transferRequested = false;}

    SmartDashboard.putNumber("Elevator Target", targetElevation);
    SmartDashboard.putNumber("Arm Target", targetAngle);
    SmartDashboard.putNumber("Elevator Height", elevation);
    SmartDashboard.putNumber("Arm Rotation", angle);

    SmartDashboard.putNumber("UA Error", motorTargets[0] - Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("DA Error", motorTargets[1] - Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble()));

    SmartDashboard.putNumber("Height over deck", elevation - arm.checkAngle(angle));
    SmartDashboard.putNumber("Encoder Reading", getEncoderPos());
    SmartDashboard.putNumber("Offset", angle - getEncoderPos());
  }

}
