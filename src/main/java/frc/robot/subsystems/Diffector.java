// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.ProtectionDomain;
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
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;
import frc.robot.util.ArmCalculator;
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
  private final double maxAbsPos = Constants.DiffectorConstants.maxAbsPos;
  private final double turnBackThreshold = Constants.DiffectorConstants.turnBackThreshold;
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

  private double offset;
  private double reverseOffset;
  private double angle;
  private double elevation;
  private static ArmCalculator arm;
  public static boolean stowRequested = true;

  private static LocalADStar armPathFinder;
  private double mapScale = 0.1;
  private double nodeSize = 0.2;
  private int test = -360;
  private PathConstraints armPathConstraints = new PathConstraints(30, 10, 0, 0);
  private GoalEndState armEndState = new GoalEndState(0, new Rotation2d());
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
    targetAngle     = 0;

    m_diffectorUA.getConfigurator().apply(motorConfigUA);
    m_diffectorDA.getConfigurator().apply(motorConfigDA);

    m_diffectorUA.setPosition(Units.degreesToRotations((Constants.DiffectorConstants.startAngle / rotationRatio) + (Constants.DiffectorConstants.startElevation / travelRatio)));
    m_diffectorDA.setPosition(Units.degreesToRotations((Constants.DiffectorConstants.startAngle / rotationRatio) - (Constants.DiffectorConstants.startElevation / travelRatio)));

    calculateAngle();
    calculateElevation();
    cargoState = updateCargoState();

    motionMagicRequester = new MotionMagicVoltage(0);

    ensureInitialized();
    setStartPosition(fromArmRelative(elevation, angle));
    setGoalPosition(fromArmRelative(targetElevation, targetAngle, false));
    plannedPathPoints.clear();
    plannedPathPoints.add(new Translation2d(elevation, angle));
  }

  /**
   * @return Height of elevator measured from arm-axis to ground, metres
   */
  public double getElevation()
  {
    {return elevation;}
  }

  /**
   * Calculates elevator height based on motor positions
   * @return Elevator height in m
   */
  private double calculateElevation()
  {
    {return elevation = ((Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()) - Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble())) / 2) * travelRatio;}
  }

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
    {return angle = ((Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()) + Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble())) * rotationRatio) / 2;}
  
  /**
   * Arm Rotation as measured from encoder
   * @return Arm rotation, degrees anticlockwise, 0 = coral at top [0..360]
   */
  public double getEncoderPos()
  {
    // Encoder outputs [-1..1] and is geared 1:1 to the arm
    return Units.rotationsToDegrees(-encoder.getAbsolutePosition().getValueAsDouble());
  }

  double[] armPath = new double[2];
  private double projectionElevation = 0.1;
  private double projectionAngle     = 10;
  private void calculateMotorTargets()
  {
    ensureInitialized();

    targetElevation = Math.min(Constants.DiffectorConstants.maxElevation, targetElevation);

    if (targetElevation != oldElevation || targetAngle != oldAngle)
    {
      setGoalPosition(fromArmRelative(targetElevation, targetAngle, true));
      setStartPosition(fromArmRelative(elevation, angle));
      oldElevation = targetElevation;
      oldAngle = targetAngle;
    }

    if (isNewPathAvailable())
    {
      PathPlannerPath plannedPath = getCurrentPath(armPathConstraints, armEndState);

      plannedPathPoints.clear();
      if (plannedPath != null)
      { 
        List<Waypoint> plannedPathWaypoints = getCurrentPath(armPathConstraints, armEndState).getWaypoints();
        
        if (plannedPathWaypoints != null)
        {
          for (Waypoint waypoint : plannedPathWaypoints) 
          {
            plannedPathPoints.add(toArmRelative(waypoint.anchor()));
          }
        }
        else
        {
          plannedPathPoints.add(new Translation2d(arm.checkPosition(targetElevation, targetAngle), targetAngle));
        }
      }
    }

    if (plannedPathPoints.size() != 0)
    {
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
    else
    {
      goToAngle((Math.random()*720)-360);
      setElevatorTarget((Math.random()*1.3)+0.4);
    }
  }

  /**
   * Calculates the position to drive each motor to, based on the target positions for the elevator and arm
   * @param elevatorTarget Target height of the elevator carriage, metres above the ground
   * @param armTarget Target angle of the arm, degrees anticlockwise, 0 = unwound with coral at top
   * @return [motor1 target, motor2 target]
   */
  public double[] calculateMotorTargets(double elevatorTarget, double armTarget)
  {
    double[] calculatedTargets = new double[2];

    elevatorTarget = Math.min(Constants.DiffectorConstants.maxElevation, elevatorTarget);

    if (elevatorTarget < elevation && arm.checkAngle(angle) > elevatorTarget) 
    {
      elevatorTarget = Math.min(Constants.DiffectorConstants.maxElevation, elevation);
    }

    calculatedTargets[0] = (armTarget / rotationRatio) + (elevatorTarget / travelRatio);
    calculatedTargets[1] = (armTarget / rotationRatio) - (elevatorTarget / travelRatio);

    // Both motors should arive at their targets at about the same time, to ensure elevation and rotation are continious between points
    double dUA = Math.abs(calculatedTargets[0] - m_diffectorUA.getPosition().getValueAsDouble());
    double dDA = Math.abs(calculatedTargets[1] - m_diffectorDA.getPosition().getValueAsDouble());
    if (!(dUA <= 1 || dDA <= 1 || Math.ceil(dUA) == Math.ceil(dDA)))
    {
      // If one motor has to go half as far as the other, only target half of that (1/4 of the other)
      if (dDA < dUA)
      {
        calculatedTargets[1] -= (calculatedTargets[1] - m_diffectorDA.getPosition().getValueAsDouble()) * (Math.ceil(dUA - dDA) / Math.ceil(dUA));
      }
      else
      {
        calculatedTargets[0] -= (calculatedTargets[0] - m_diffectorUA.getPosition().getValueAsDouble()) * (Math.ceil(dDA - dUA) / Math.ceil(dDA));
      }
    }

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

  /**
   * Sets the Diffector arm to rotate the shortest path to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goShortest(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(angle, 360), -180, 180);

    if (angle + offset > maxAbsPos)
      {targetAngle = (angle + offset - 360);}

    else if (angle + offset < -maxAbsPos)
      {targetAngle = (angle + offset + 360);}

    else
      {targetAngle = (angle + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Clockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goClockwise(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(angle, 360), -360, 0);

    if (angle + offset > maxAbsPos)
      {targetAngle = (angle + offset - 360);}

    else if (angle + offset < -maxAbsPos)
      {targetAngle = (angle + offset + 360);}

    else
      {targetAngle = (angle + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Anticlockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goAnticlockwise(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(angle, 360), 0, 360);

    if (angle + offset > maxAbsPos)
      {targetAngle = (angle + offset - 360);}

    else if (angle + offset < -maxAbsPos)
      {targetAngle = (angle + offset + 360);}

    else
      {targetAngle = (angle + offset);}
  }

  /**
   * Sets the Diffector arm to rotate the safest path to the target angle, with protection against over-rotation. 
   * Below a threshold will go shortest path, otherwise will minimise total rotations
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goToAngle(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(angle, 360), -180, 180);

    if (Math.abs(offset) >= turnBackThreshold)
    {
      reverseOffset = offset - Math.copySign(360, offset);

      if (Math.abs(angle + offset) > Math.abs(angle + reverseOffset))
        {targetAngle = (angle + reverseOffset);}
      
      else 
        {targetAngle = (angle + offset);}
    }
    else if (angle + offset > maxAbsPos)
      {targetAngle = (angle + offset - 360);}

    else if (angle + offset < -maxAbsPos)
      {targetAngle = (angle + offset + 360);}

    else
      {targetAngle = (angle + offset);}
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

  @Override
  public void periodic() 
  { 
    calculateAngle();
    calculateElevation();
    cargoState = updateCargoState();

    calculateMotorTargets();

    m_diffectorUA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[0])).withSlot(0));//getSlot()));
    m_diffectorDA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[1])).withSlot(0));//getSlot()));
    // TODO: Ensure both motors take the same time to reach their respective targets

    
    if (stowRequested && (Math.abs(angle) > stowThreshold || Math.abs(targetAngle) > stowThreshold))
     {stowRequested = false;}

    SmartDashboard.putNumber("Elevator Target", targetElevation);
    SmartDashboard.putNumber("Arm Target", targetAngle);
    SmartDashboard.putNumber("Elevator Height", elevation);
    SmartDashboard.putNumber("Arm Rotation", angle);
    SmartDashboard.putBoolean("Existance", true);

    SmartDashboard.putNumber("UA Error", motorTargets[0] - Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("DA Error", motorTargets[1] - Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble()));

  }


  
/*
 * The following functions exist in place of the "Pathfinding" file from the Pathplanner library.
 * 
 * A custom version of "LocalADStar" has been created, with an overload constructor that allows for a
 * custom navGrid file path to be specified.
 * 
 * We have then mapped out the valid (z,φ) space of the Diffector arm, presented as a (x,y) navGrid.
 * 
 * By using this, we are (hopefully) able to use the pathfinding algorithms from Pathplanner to
 * produce an optimal path for the Diffector arm to follow.
 */

 /**
  * Converts from arm-relative coordinates to the coordinates used by the AD* system
  * @param armElevationRotation metres over ground | total degrees anticlockwise
  * @return pathfinderXY -> arm state-space mapping
  */
 public Translation2d fromArmRelative(Translation2d armElevationRotation)
 {
    return new Translation2d
    (
      (Math.max((armElevationRotation.getX() - Constants.DiffectorConstants.minElevation), 0) / mapScale),
      (armElevationRotation.getY() + maxAbsPos) * mapScale
    );
 }

 /**
  * Converts from arm-relative coordinates to the coordinates used by the AD* system 
  * @param armElevation metres over ground
  * @param armRotation total degrees anticlockwise
  * @param protected ensure the elevation is safe for the given rotation (default false)
  * @return pathfinderXY -> arm state-space mapping
  */
  public Translation2d fromArmRelative(double armElevation, double armRotation)
  {
    {return fromArmRelative(new Translation2d(armElevation, armRotation));}
  }

  /**
  * Converts from arm-relative coordinates to the coordinates used by the AD* system 
  * @param armElevation metres over ground
  * @param armRotation total degrees anticlockwise
  * @param protect ensure the elevation is safe for the given rotation (default false)
  * @return pathfinderXY -> arm state-space mapping
  */
  public Translation2d fromArmRelative(double armElevation, double armRotation, boolean protect)
  {
    if (protect)
      {return fromArmRelative(new Translation2d(arm.checkPosition(armElevation, armRotation), armRotation));}
    else
      {return fromArmRelative(new Translation2d(armElevation, armRotation));}
  }

 /**
  * Converts from the coordinates used by the AD* system to arm-relative coordinates
  * @param pathfinderXY arm state-space mapping
  * @return armElevationRotation -> metres over ground | total degrees anticlockwise
  */
 public Translation2d toArmRelative(Translation2d pathfinderXY)
 {
    return new Translation2d
    (
      (pathfinderXY.getX() * mapScale) + Constants.DiffectorConstants.minElevation,
      (pathfinderXY.getY() / mapScale) - maxAbsPos
    );
 }

  /** Ensure that a pathfinding implementation has been chosen. If not, set it to the default. */
  public static void ensureInitialized() 
  {
    if (armPathFinder == null) 
    {
      // Hasn't been initialized yet, use the default implementation
      armPathFinder = new LocalADStar("armplanner/diffector_navgrid.json");
    }
  }

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  public static boolean isNewPathAvailable() 
    {return armPathFinder.isNewPathAvailable();}

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  public static PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) 
    {return armPathFinder.getCurrentPath(constraints, goalEndState);}

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  public static void setStartPosition(Translation2d startPosition) 
    {armPathFinder.setStartPosition(startPosition);}

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  public static void setGoalPosition(Translation2d goalPosition) 
    {armPathFinder.setGoalPosition(goalPosition);}

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path if the robot is now within an obstacle.
   */
  public static void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) 
    {armPathFinder.setDynamicObstacles(obs, currentRobotPos);}
}
