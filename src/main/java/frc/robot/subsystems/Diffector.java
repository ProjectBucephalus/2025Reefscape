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
import frc.robot.constants.Constants.DiffectorConstants;
import frc.robot.constants.Constants.DiffectorConstants.IKGeometry;
import frc.robot.constants.IDConstants;
import frc.robot.util.ArmCalculator;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;

public class Diffector extends SubsystemBase 
{
  private boolean eStop;

  public enum CargoStates{DEFAULT, SPRING}
  private CargoStates cargoState;

  private boolean manualControl;
  private double manualElevation;
  private double manualRotation;

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
  private Translation2d relativeTarget;

  private double angle;
  private double elevation;
  private Translation2d armPosition;

  public static ArmCalculator arm;
  public static boolean transferRequested = false;
  public static boolean stowRequested = true;

  private double projectionElevation = IKGeometry.projectionElevation;
  private double projectionAngle     = IKGeometry.projectionAngle;
  //private PathConstraints armPathConstraints = new PathConstraints(1, 1, 0, 0);
  //private GoalEndState armEndState = new GoalEndState(0, Rotation2d.kZero);
  private static ArrayList<Translation2d> plannedPathPoints = new ArrayList<Translation2d>();

  private int calibrationCounter = 0;

  /** Creates a new Diffector. */
  public Diffector() 
  {
    eStop = false;
    SmartDashboard.putBoolean("Diffector E-Stop", eStop);
    manualControl = false;
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
    oldTarget       = targetPosition;
    relativeTarget  = targetPosition;

    m_diffectorUA.getConfigurator().apply(motorConfigUA);
    m_diffectorDA.getConfigurator().apply(motorConfigDA);

    m_diffectorUA.setPosition(Units.degreesToRotations((targetPosition.getY() / rotationRatio) + (targetPosition.getX() / travelRatio)));
    m_diffectorDA.setPosition(Units.degreesToRotations((targetPosition.getY() / rotationRatio) - (targetPosition.getX() / travelRatio)));
    
    motorTargets = calculateMotorTargets(targetPosition);

    calculatePosition();
    cargoState = updateCargoState();

    motionMagicRequester = new MotionMagicVoltage(0);

    //ArmPathPlanner.ensureInitialized();
    //ArmPathPlanner.setStartPosition(ArmPathPlanner.fromArmRelative(armPosition));
    //ArmPathPlanner.setGoalPosition(ArmPathPlanner.fromArmRelative(targetPosition, false));
    plannedPathPoints.clear();
    plannedPathPoints.add(targetPosition);

    SmartDashboard.putBoolean("Overide: Calibrate Arm", false);
    SmartDashboard.putBoolean("Overide: Arm At Target", false);
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
    armPosition = new Translation2d(elevation, angle);
    
    if 
    (
      relativeTarget.equals(DiffectorConstants.startPosition) ||
      relativeTarget.equals(DiffectorConstants.coralTransferPosition) ||
      relativeTarget.equals(DiffectorConstants.algaeIntakePosition) ||
      relativeTarget.equals(DiffectorConstants.climbPosition)
    )
    {
      if (elevation < targetElevation - DiffectorConstants.elevationTolerance)
        {eStop = true;}
    }
    else if (elevation < arm.checkPosition(armPosition) - DiffectorConstants.elevationTolerance)
      {eStop = true;}

    if 
    (
      atPosition() &&
      (relativeTarget.equals(DiffectorConstants.algaeStowPosition) ||
      relativeTarget.equals(DiffectorConstants.coralStowPosition) ||
      relativeTarget.equals(DiffectorConstants.coralIntakePosition) ||
      relativeTarget.equals(DiffectorConstants.algaeTransferPosition))
    )
    {
      calibrationCounter++;
      if (calibrationCounter > 10) 
      {
        SmartDashboard.putBoolean("Overide: Calibrate Arm", true);
      }
    }
    else
    {
      calibrationCounter = 0;
    }

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
    {return Conversions.mod(angle, 360);}

  private void calculatePath()
  {
    targetPosition = new Translation2d(targetElevation, targetAngle);
    relativeTarget = new Translation2d(targetElevation, Conversions.mod(targetAngle, 360));

    if (!targetPosition.equals(oldTarget))
    {
      if 
      (
        !(
          MathUtil.isNear(RobotContainer.swerveState.Pose.getTranslation().getX(), FieldUtils.fieldLength / 2, IKGeometry.bargeSafetyWidth) &&
          targetPosition.getX() > IKGeometry.bargeSafetyHeight
        )
      )
    
      oldTarget = targetPosition;

      plannedPathPoints = arm.pathfindArm(targetPosition, armPosition);
      //SmartDashboard.putNumberArray("pathDump", plannedPathPoints.stream().mapMultiToDouble((point, consumer) -> {consumer.accept(point.getX()); consumer.accept(point.getY());}).toArray());
    }

    if (plannedPathPoints.size() != 0)
    {
      //SmartDashboard.putNumberArray("target Point", new double[]{plannedPathPoints.get(0).getX(), plannedPathPoints.get(0).getY()});
      motorTargets = calculateMotorTargets(plannedPathPoints.get(0));

      if (atPosition(plannedPathPoints.get(0)))
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

  /**
   * Returns true if the diffector is at the given position
   * @param checkTarget target elevation/rotation to check against
   */
  public boolean atPosition(Translation2d checkTarget)
  {
    return
      Math.abs(elevation - checkTarget.getX()) < DiffectorConstants.elevationTolerance &&
      Math.abs(angle - checkTarget.getY()) < DiffectorConstants.angleTolerance;
  }

  /** Returns true if the diffector is safely in climb position */
  public boolean climbReady()
    {return atPosition(DiffectorConstants.climbPosition);}

  /** 
   * Sets the Diffector arm to unwind to starting position 
   * @return Safe to stow
   */
  public boolean unwind()
  {
    targetAngle = Constants.DiffectorConstants.startPosition.getY();
    return (stowRequested = Math.abs(angle) < stowThreshold);
  }

  public Translation2d getRelativeTarget()
    {return relativeTarget;}

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
      case DEFAULT: return 0;
      case SPRING: return 1;
      default: return 0;
    }
  }

  private CargoStates updateCargoState()
  {
   // Default state, should never be reached
   {return CargoStates.DEFAULT;}
  }

  public void setManualDiffectorValues(double newManualElevation, double newManualRotation)
  {
    if (newManualElevation != 0 || newManualRotation != 0) 
      {manualControl = true;}

    manualElevation = newManualElevation;
    manualRotation = newManualRotation;
  }

  public void goToAngle(double newTarget) 
  {
    if (RobotContainer.algae)
    {
      if (getRelativeRotation() < 180 && Conversions.mod(newTarget, 360) > 180)
        {targetAngle = arm.goAnticlockwise(newTarget, angle);} // Going Anticlockwise to take held Algae over robot

      else if (getRelativeRotation() > 180 && Conversions.mod(newTarget, 360) < 180)
        {targetAngle = arm.goClockwise(newTarget, angle);} // Going Clockwise to take held Algae over robot
        
      else
        {targetAngle = arm.goShortest(newTarget, angle);}
    }
    else
      {targetAngle = arm.goToAngle(newTarget, angle);}
  }

  /**
   * WARNING: Updates the Diffector motor positions to match the input arm position, 
   * only use this when absolutely necessary!
   * @param setElevation Height of the centre of rotation above the ground, metres
   * @param setAngle Angle of the arm, total degrees Anticlockwise
   * @return True if existing position is near set position
   */
  public boolean positionOveride(double setElevation, double setAngle)
  {
    setElevation = Conversions.clamp(setElevation, DiffectorConstants.minZ, DiffectorConstants.maxZ);
    m_diffectorUA.setPosition(Units.degreesToRotations((setAngle / rotationRatio) + (setElevation / travelRatio)));
    m_diffectorDA.setPosition(Units.degreesToRotations((setAngle / rotationRatio) - (setElevation / travelRatio)));

    return (!MathUtil.isNear(elevation, setElevation, DiffectorConstants.elevationTolerance) || !MathUtil.isNear(angle, setAngle, DiffectorConstants.angleTolerance));
  }

  @Override
  public void periodic() 
  { 
    if (SmartDashboard.getBoolean("Overide: Calibrate Arm", false))
    {
      positionOveride(elevation, getEncoderPos());
      SmartDashboard.putBoolean("Overide: Calibrate Arm", false);
    }
    
    if (SmartDashboard.getBoolean("OVERIDE MODE", false))
    {
      if (SmartDashboard.getBoolean("Overide: Arm At Target", false))
      {
        positionOveride(targetElevation, targetAngle);
        plannedPathPoints.clear();
        SmartDashboard.putBoolean("Overide: Arm At Target", false);
      }
    }

    calculatePosition();
    cargoState = updateCargoState();
    
    if 
    (
      Math.abs(m_diffectorUA.getTorqueCurrent().getValueAsDouble()) > Constants.DiffectorConstants.motorStallCurrent ||
      Math.abs(m_diffectorDA.getTorqueCurrent().getValueAsDouble()) > Constants.DiffectorConstants.motorStallCurrent
    )
    {
      eStop = true;
      SmartDashboard.putBoolean("Diffector E-Stop", eStop);
    }
    else
      {eStop = SmartDashboard.getBoolean("Diffector E-Stop", false);}
    
    if (eStop)
    {
      m_diffectorUA.set(0);
      m_diffectorDA.set(0);
      eStop = SmartDashboard.getBoolean("Diffector E-Stop", true);
    }
    else
    {
      if (manualControl)
      {
        if (manualElevation != 0) 
        {
          if 
          (
            (manualElevation < 0 && arm.checkAngle(angle) > elevation - projectionElevation) || 
            (manualElevation > 0 && elevation + projectionElevation > DiffectorConstants.maxZ)
          )
            {manualElevation = 0;}
        }

        if (manualRotation != 0)
        {
          if (arm.checkAngle(angle + Math.copySign(projectionAngle, manualRotation)) > elevation) 
            {manualRotation = 0;}
        }

        if (manualElevation == 0 && manualRotation == 0)
        {
          goToAngle(angle);
          setElevationTarget(elevation);
          manualControl = false;
        }
      }

      if (manualControl)
      {
        m_diffectorUA.setVoltage((manualRotation * Constants.Control.manualDiffectorRotationScalar) + (manualElevation * Constants.Control.manualDiffectorElevationScalar));
        m_diffectorDA.setVoltage((manualRotation * Constants.Control.manualDiffectorRotationScalar) - (manualElevation * Constants.Control.manualDiffectorElevationScalar));
      }
      else
      {
        calculatePath();

        m_diffectorUA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[0])).withSlot(0));//getSlot()));
        m_diffectorDA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[1])).withSlot(0));//getSlot()));
      }
      
      if (transferRequested && !MathUtil.isNear(180, getRelativeRotation(), DiffectorConstants.angleTolerance))
        {transferRequested = false;}
        
      if (transferRequested && !MathUtil.isNear(0, angle, DiffectorConstants.angleTolerance))
        {stowRequested = false;}

    }
    SmartDashboard.putNumber("Elevator Target", targetElevation);
    SmartDashboard.putNumber("Arm Target", targetAngle);
    SmartDashboard.putNumber("Elevator Height", elevation);
    SmartDashboard.putNumber("Arm Rotation", angle);
    SmartDashboard.putNumber("Relative Angle Target", relativeTarget.getY());

    SmartDashboard.putNumber("UA Error", motorTargets[0] - Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("DA Error", motorTargets[1] - Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble()));

    SmartDashboard.putNumber("Height over deck", elevation - arm.checkAngle(angle));
    SmartDashboard.putNumber("Encoder Reading", getEncoderPos());
    SmartDashboard.putNumber("Offset", angle - getEncoderPos());
  }
}
