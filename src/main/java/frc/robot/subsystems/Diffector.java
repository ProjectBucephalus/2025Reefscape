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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.util.SD;
import frc.robot.util.SD.Key;

public class Diffector extends SubsystemBase 
{
  private boolean eStop;

  private boolean springState = false;

  private boolean manualControl;
  private double  manualElevation;
  private double  manualRotation;

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

  private static ArmCalculator arm;

  private double projectionElevation = IKGeometry.projectionElevation;
  private double projectionAngle     = IKGeometry.projectionAngle;
  private ArrayList<Translation2d> plannedPathPoints = new ArrayList<Translation2d>();

  private int calibrationCounter = 0;

  /** Creates a new Diffector. */
  public Diffector() 
  {
    eStop = false;
    SD.init(Key.DIFF_ESTOP);
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
    
    motorTargets = calculateMotorTargets(targetPosition);

    if (Conversions.mod(getEncoderPos(), 360) > Constants.DiffectorConstants.angleTolerance && Conversions.mod(getEncoderPos(), 360) < 360 - Constants.DiffectorConstants.angleTolerance) 
    {
      eStop = true;
    }
    positionOveride(targetElevation, getEncoderPos());

    calculatePosition();

    updateSpringState();

    motionMagicRequester = new MotionMagicVoltage(0);

    plannedPathPoints.clear();
    plannedPathPoints.add(targetPosition);

    SD.init(Key.CALIBRATE_DIFF);
    SD.init(Key.CALIBRATE_DIFF_TARGET);
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
    
    if (DiffectorConstants.lowDiffectorPositions.stream().anyMatch(position -> relativeTarget.equals(position)))
    {
      if (elevation < targetElevation - DiffectorConstants.elevationTolerance)
        {eStop = true;}
    }
    else if (elevation < arm.checkPosition(armPosition) - DiffectorConstants.elevationTolerance && !manualControl)
      {eStop = true;}

    if 
    (
      atPosition()// &&
      //(relativeTarget.equals(DiffectorConstants.algaeStowPosition) ||
      //relativeTarget.equals(DiffectorConstants.coralStowPosition) ||
      //relativeTarget.equals(DiffectorConstants.coralIntakePosition) ||
      //relativeTarget.equals(DiffectorConstants.algaeTransferPosition))
    )
    {
      calibrationCounter++;
      if (calibrationCounter == DiffectorConstants.calibrationDelay) 
      {
        SD.put(Key.CALIBRATE_DIFF, true);
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
    }

    if (plannedPathPoints.size() != 0)
    {
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
    if (MathUtil.isNear(angle, target.getY(), DiffectorConstants.angleTolerance))
    { // If movement is only elevation, use elevation acceleration limits
      m_diffectorUA.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicAcceleration(DiffectorConstants.diffectorElevationAcceleration));
      m_diffectorDA.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicAcceleration(DiffectorConstants.diffectorElevationAcceleration));
    }
    else
    { // If movement includes rotation, use rotation acceleration limits
      m_diffectorUA.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicAcceleration(DiffectorConstants.diffectorRotationAcceleration));
      m_diffectorDA.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicAcceleration(DiffectorConstants.diffectorRotationAcceleration));
    }

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
   * Returns true if the diffector is at the given absolute position
   * @param checkTarget target elevation/rotation to check against
   */
  public boolean atPosition(Translation2d checkTarget)
  {
    return
      Math.abs(elevation - checkTarget.getX()) < DiffectorConstants.elevationTolerance &&
      Math.abs(angle - checkTarget.getY()) < DiffectorConstants.angleTolerance;
  }

  /**
   * Returns true if the diffector is at the given relative position
   * @param checkTarget target elevation/rotation to check against
   */
  public boolean atRelativePosition(Translation2d checkTarget)
  {
    return
      Math.abs(elevation - checkTarget.getX()) < DiffectorConstants.elevationTolerance &&
      Math.abs(getRelativeRotation() - Conversions.mod(checkTarget.getY(), 360)) < DiffectorConstants.angleTolerance;
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
    manualControl = false;
    targetAngle = Constants.DiffectorConstants.startPosition.getY();
    return Math.abs(angle) < stowThreshold;
  }

  public void setElevationTarget(double newTarget)
  {
    manualControl = false;
    targetElevation = Conversions.clamp(newTarget, Constants.DiffectorConstants.minZ, Constants.DiffectorConstants.maxZ);
  }

  /** Returns the ID of the motor control slot to use */
  private int getSlot()
  {
    return springState ? 1 : 0;
  }

  private boolean updateSpringState()
   {return springState = false;}

  public void setManualDiffectorValues(double newManualElevation, double newManualRotation)
  {
    if (newManualElevation != 0 || newManualRotation != 0) 
      {manualControl = true;}

    manualElevation = newManualElevation;
    manualRotation = newManualRotation;
  }

  public void goToAngle(double newTarget) 
  {
    manualControl = false;
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

  public void setTargetPosition(Translation2d targetPosition)
  {
    setElevationTarget(targetPosition.getX());
    goToAngle(targetPosition.getY());
  }

  public Command moveToCommand(Translation2d targetPosition)
  {
    return 
    runOnce(() -> setTargetPosition(targetPosition))
    .onlyIf(() -> !RobotState.isDisabled() || SD.getBoolean(Key.OVERIDE))
    .ignoringDisable(true);
  }

  public Command moveAndWaitCommand(Translation2d targetPosition)
  {
    return moveToCommand(targetPosition).andThen(Commands.waitUntil(() -> atPosition()));
  }

  public Command algaeIntakePosCommand(Translation2d robotPos, boolean level2, int nearestReefFace)
  {
    boolean portReefFace = (nearestReefFace == 5 || nearestReefFace == 6);

    Translation2d target = 
    level2 
    ?
    portReefFace ? Constants.DiffectorConstants.algae2PortPosition : Constants.DiffectorConstants.algae2StbdPosition
    :
    portReefFace ? Constants.DiffectorConstants.algae3PortPosition : Constants.DiffectorConstants.algae3StbdPosition;

    return moveAndWaitCommand(target);
  }

  public Command algaeIntakePosCommand(Translation2d robotPos, boolean level2)
  {
    return algaeIntakePosCommand(robotPos, level2, FieldUtils.getNearestReefFace(robotPos));
  }

  public Command algaeIntakePosCommand(boolean level2)
  {
    return algaeIntakePosCommand(RobotContainer.swerveState.Pose.getTranslation(), level2);
  }

  public Command algaeIntakePosCommand(Translation2d robotPos)
  {
    int nearestReefFace = FieldUtils.getNearestReefFace(robotPos);
    return algaeIntakePosCommand(robotPos, nearestReefFace % 2 == 0, nearestReefFace);
  }

  public Command algaeIntakePosCommand()
  {
    return algaeIntakePosCommand(RobotContainer.swerveState.Pose.getTranslation());
  }

  public Command coralScorePosInstantCommand(Translation2d robotPos, int level)
  {
    int nearestReefFace = FieldUtils.getNearestReefFace(robotPos);
    boolean portReefFace = (nearestReefFace == 5 || nearestReefFace == 6);

    Translation2d target = 
    switch (level)
    {
      case 4 -> portReefFace ? Constants.DiffectorConstants.coral4PortPosition : Constants.DiffectorConstants.coral4StbdPosition;

      case 3 -> portReefFace ? Constants.DiffectorConstants.coral3PortPosition : Constants.DiffectorConstants.coral3StbdPosition;

      case 2 -> portReefFace ? Constants.DiffectorConstants.coral2PortPosition : Constants.DiffectorConstants.coral2StbdPosition;

      case 1 -> portReefFace ? Constants.DiffectorConstants.coral1PortPosition : Constants.DiffectorConstants.coral1StbdPosition;

      default -> Constants.DiffectorConstants.coralStowPosition;
    };

    return moveToCommand(target);
  }

  public Command coralScorePosCommand(Translation2d robotPos, int level)
  {
    return coralScorePosCommand(robotPos, level).andThen(Commands.waitUntil(() -> atPosition()));
  }

  public Command coralScorePosCommand(int level)
  {
    return coralScorePosCommand(RobotContainer.swerveState.Pose.getTranslation(), level);
  }

  @Override
  public void periodic() 
  { 
    if (SD.getBoolean(Key.CALIBRATE_DIFF))
    {
      positionOveride(elevation, getEncoderPos());
      SD.put(Key.CALIBRATE_DIFF, false);
    }
    
    if (SD.getBoolean(Key.OVERIDE))
    {
      if (SD.getBoolean(Key.CALIBRATE_DIFF_TARGET))
      {
        positionOveride(targetElevation, targetAngle);
        plannedPathPoints.clear();
        SD.put(Key.CALIBRATE_DIFF_TARGET, false);
      }
    }

    calculatePosition();
    updateSpringState();
    
    if 
    (
      Math.abs(m_diffectorUA.getTorqueCurrent().getValueAsDouble()) > Constants.DiffectorConstants.motorStallCurrent ||
      Math.abs(m_diffectorDA.getTorqueCurrent().getValueAsDouble()) > Constants.DiffectorConstants.motorStallCurrent
    )
    {
      eStop = true;
      SD.put(Key.DIFF_ESTOP, true);
    }
    else
      {eStop = SD.getBoolean(Key.DIFF_ESTOP);}
    
    if (eStop)
    {
      m_diffectorUA.set(0);
      m_diffectorDA.set(0);
      eStop = SD.getBoolean(Key.DIFF_ESTOP);
    }
    else
    {
      if (manualControl)
      {
        if (manualRotation != 0)
        {
          if (arm.checkAngle(angle + Math.copySign(projectionAngle, manualRotation)) > elevation) 
            {manualRotation = 0;}
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

        m_diffectorUA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[0])));//.withSlot(getSlot()));
        m_diffectorDA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[1])));//.withSlot(getSlot()));
      }
    }
    SD.put(Key.DIFF_ELEVATION_TARGET, targetElevation);
    SD.put(Key.DIFF_ANGLE_TARGET, targetAngle);
    SD.put(Key.DIFF_ELEVATION, elevation);
    SD.put(Key.DIFF_ANGLE, angle);

    SD.put(Key.DIFF_UA_ER, motorTargets[0] - Units.rotationsToDegrees(m_diffectorUA.getPosition().getValueAsDouble()));
    SD.put(Key.DIFF_DA_ER, motorTargets[1] - Units.rotationsToDegrees(m_diffectorDA.getPosition().getValueAsDouble()));

    SD.put(Key.DIFF_HEIGHT, elevation - arm.checkAngle(angle));
    SD.put(Key.SENSOR_DIFF_ANGLE, getEncoderPos());
    SD.put(Key.DIFF_ANGLE_ER, angle - getEncoderPos());
  }
}
