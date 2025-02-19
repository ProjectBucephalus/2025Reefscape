// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;
import frc.robot.util.ArmCalculator;
import frc.robot.util.ArmPathPlanner;

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

  private double targetElevation;
  private double targetAngle;
  private double oldElevation;
  private double oldAngle;

  public double angle;
  private double elevation;
  public static ArmCalculator arm;
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

    targetElevation = Constants.DiffectorConstants.startElevation;
    targetAngle     = Constants.DiffectorConstants.startAngle;

    m_diffectorUA.getConfigurator().apply(motorConfigUA);
    m_diffectorDA.getConfigurator().apply(motorConfigDA);

    m_diffectorUA.setPosition(Units.degreesToRotations((Constants.DiffectorConstants.startAngle / rotationRatio) + (Constants.DiffectorConstants.startElevation / travelRatio)));
    m_diffectorDA.setPosition(Units.degreesToRotations((Constants.DiffectorConstants.startAngle / rotationRatio) - (Constants.DiffectorConstants.startElevation / travelRatio)));
    
    motorTargets = calculateMotorTargets(targetElevation, targetAngle);

    calculateAngle();
    calculateElevation();
    cargoState = updateCargoState();

    motionMagicRequester = new MotionMagicVoltage(0);

    ArmPathPlanner.ensureInitialized();
    ArmPathPlanner.setStartPosition(ArmPathPlanner.fromArmRelative(elevation, angle));
    ArmPathPlanner.setGoalPosition(ArmPathPlanner.fromArmRelative(targetElevation, targetAngle, false));
    plannedPathPoints.clear();
    plannedPathPoints.add(new Translation2d(elevation, angle));
  }

  /**
   * @return Height of elevator measured from arm-axis to ground, metres
   */
  public double getElevation()
    {return elevation;}

  /**
   * Calculates elevator height based on motor positions
   * @return Elevator height in m
   */
  private double calculateElevation()
    {return elevation = ((Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()) - Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble())) / 2) * travelRatio;}

  /**
   * @return Arm rotation, degrees anticlockwise as seen from the front, 0 = coral at top
   */
  public double getAngle()
    {return angle;}
  
  /**
   * Calculates arm rotation based on motor positions
   * @return Arm rotation, degrees anticlockwise, 0 = coral at top
   */
  private double calculateAngle()
  {
    angle = ((Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()) + Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble())) * rotationRatio) / 2;
    if (Math.floor(angle) != Math.floor(getEncoderPos()))
    {
      angle = getEncoderPos();
      m_diffectorUA.setPosition(Units.degreesToRotations((angle / rotationRatio) + (elevation / travelRatio)));
      m_diffectorDA.setPosition(Units.degreesToRotations((angle / rotationRatio) - (elevation / travelRatio)));
    }
    return angle;
  }
  
  /**
   * Arm Rotation as measured from encoder
   * @return Arm rotation, degrees anticlockwise, 0 = coral at top
   */
  public double getEncoderPos()
  {
    // Encoder outputs is geared 1:1 to the arm, so output is inverted
    return Units.rotationsToDegrees(-encoder.getPosition().getValueAsDouble());
  }

  private void calculatePath()
  {
    targetElevation = Math.min(Constants.DiffectorConstants.maxElevation, targetElevation);

    if (targetElevation != oldElevation || targetAngle != oldAngle)
    {
      targetElevation = arm.checkPosition(targetElevation, targetAngle);
      //ArmPathPlanner.setGoalPosition(ArmPathPlanner.fromArmRelative(targetElevation, targetAngle, true));
      //ArmPathPlanner.setStartPosition(ArmPathPlanner.fromArmRelative(elevation, angle));
      oldElevation = targetElevation;
      oldAngle = targetAngle;

      plannedPathPoints = arm.pathfindArm(targetElevation, targetAngle, elevation, angle);
    }

    /*if (ArmPathPlanner.isNewPathAvailable())
    {      
      PathPlannerPath plannedPath = ArmPathPlanner.getCurrentPath(armPathConstraints, armEndState);

      plannedPathPoints.clear();
      if (plannedPath != null)
      { 
        List<Waypoint> plannedPathWaypoints = ArmPathPlanner.getCurrentPath(armPathConstraints, armEndState).getWaypoints();
        
        if (plannedPathWaypoints != null)
        {
          plannedPathPoints.addAll(plannedPathWaypoints.stream().map(waypoint -> ArmPathPlanner.toArmRelative(waypoint.anchor())).toList());
        }
        else
        {
          plannedPathPoints.add(new Translation2d(arm.checkPosition(targetElevation, targetAngle), targetAngle));
        }
      }
    }*/

    if (plannedPathPoints.size() != 0)
    {
      SmartDashboard.putNumberArray("target Point", new double[]{plannedPathPoints.get(0).getX(), plannedPathPoints.get(0).getY()});
      motorTargets = calculateMotorTargets(plannedPathPoints.get(0).getX(), plannedPathPoints.get(0).getY());

      if 
      (
        Math.abs(plannedPathPoints.get(0).getX() - elevation) <= projectionElevation &&
        Math.abs(plannedPathPoints.get(0).getY() - angle) <= projectionAngle
      )
      {
        plannedPathPoints.remove(0);
      }
    }
  }

  /**
   * Calculates the position to drive each motor to, based on the target positions for the elevator and arm
   * @param elevationTarget Target height of the elevator carriage, metres above the ground
   * @param angleTarget Target angle of the arm, degrees anticlockwise, 0 = unwound with coral at top
   * @return [motor1 target, motor2 target]
   */
  public double[] calculateMotorTargets(double elevationTarget, double angleTarget)
  {
    double[] calculatedTargets = new double[2];

    elevationTarget = Math.min(Constants.DiffectorConstants.maxElevation, elevationTarget);

    if (elevationTarget < elevation && arm.checkAngle(angle) > elevationTarget) 
    {
      elevationTarget = Math.min(Constants.DiffectorConstants.maxElevation, elevation);
    }

    calculatedTargets[0] = (angleTarget / rotationRatio) + (elevationTarget / travelRatio);
    calculatedTargets[1] = (angleTarget / rotationRatio) - (elevationTarget / travelRatio);

    return calculatedTargets;
  }

  /** Returns true if the diffector is at its current target angle */
  public boolean armAtAngle()
    {return Math.abs(angle - targetAngle) < Constants.DiffectorConstants.angleTolerance;}

  /** Returns true if the diffector is at its current target elevation */
  public boolean elevatorAtElevation()
    {return Math.abs(elevation - targetElevation) < Constants.DiffectorConstants.elevationTolerance;}

  /** Returns true if the diffector is safely in climb position */
  public boolean climbReady()
    {return (targetElevation == Constants.DiffectorConstants.climbElevation && elevatorAtElevation());}

  /** 
   * Sets the Diffector arm to unwind to starting position 
   * @return Safe to stow
   */
  public boolean unwind()
  {
    targetAngle = Constants.DiffectorConstants.startAngle;
    return (stowRequested = Math.abs(angle) < stowThreshold);
  }

  public double getArmTarget()
    {return targetAngle;}

  public void setElevatorTarget(double newTarget)
    {targetElevation = Math.max(newTarget, Constants.DiffectorConstants.minElevation);}

  public double getElevatorTarget()
    {return targetElevation;}

  public boolean safeToMoveCoral()
  {
    return Constants.DiffectorConstants.coralElevatorLowTheshold < elevation 
    && elevation < Constants.DiffectorConstants.coralElevatorHighThreshold;
  }

  public boolean safeToMoveAlgae()
  {
    return Constants.DiffectorConstants.algaeElevatorLowTheshold < elevation 
    && elevation < Constants.DiffectorConstants.algaeElevatorHighThreshold;
  }

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
    {
      m_diffectorUA.set(input);
    }
    else
    {
      m_diffectorDA.set(input);
    }
  }

  public void goToAngle(double newTarget) 
    {targetAngle = arm.goToAngle(newTarget);}

  @Override
  public void periodic() 
  { 
    calculateElevation();
    calculateAngle();
    cargoState = updateCargoState();

    calculatePath();

    m_diffectorUA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[0])).withSlot(0));//getSlot()));
    m_diffectorDA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[1])).withSlot(0));//getSlot()));
    
    if (stowRequested && (Math.abs(angle) > stowThreshold || Math.abs(targetAngle) > stowThreshold))
     {stowRequested = false;}

    SmartDashboard.putNumber("Elevator Target", targetElevation);
    SmartDashboard.putNumber("Arm Target", targetAngle);
    SmartDashboard.putNumber("Elevator Height", elevation);
    SmartDashboard.putNumber("Arm Rotation", angle);

    SmartDashboard.putNumber("UA Error", motorTargets[0] - Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("DA Error", motorTargets[1] - Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble()));

    SmartDashboard.putNumber("Height over deck", elevation - arm.checkAngle(angle));
    SmartDashboard.putNumber("Encoder Reading", getEncoderPos());
  }

}
