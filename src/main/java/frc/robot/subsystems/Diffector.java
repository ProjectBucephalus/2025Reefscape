// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.DiffectorGeometry;
import frc.robot.constants.Constants.DiffectorConstants;
import frc.robot.constants.Constants.DiffectorConstants.Presets;
import frc.robot.constants.IDConstants;
import frc.robot.constants.MechanismConstants.DiffectorConfigs;
import frc.robot.util.ArmCalculator;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;
import frc.robot.util.SD;

public class Diffector extends SubsystemBase 
{
  private boolean eStop;
  
  private boolean manualControl;
  private double  manualElevation;
  private double  manualRotation;
  
  private final MotionMagicVoltage motionMagicRequester;
  private final double rotationRatio;
  private final double travelRatio;
  private final TalonFXConfiguration motorConfigUA = DiffectorConfigs.getMotorConfigs();
  private final TalonFXConfiguration motorConfigDA = DiffectorConfigs.getMotorConfigs();
  private final double stowThreshold = DiffectorGeometry.angleTolerance;
  
  /* Name is effect of motor when running anticlockwise/positive (e.g. elevator Up, arm Anticlockwise) */
  /** starboard-side motor(?), forward direction drives carriage up and anticlockwise */
  private static TalonFX m_UA;
  /** port-side motor(?), forward direction drives carriage down and anticlockwise */
  private static TalonFX m_DA;
  private CANcoder io_Rotation;
  private AnalogPotentiometer io_Elevation;

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

  private double projectionElevation = DiffectorGeometry.projectionElevation;
  private double projectionAngle     = DiffectorGeometry.projectionAngle;
  private ArrayList<Translation2d> plannedPathPoints = new ArrayList<Translation2d>();

  private int calibrationCounter = 0;

  /** Creates a new Diffector. */
  public Diffector() 
  {
    eStop = false;
    SD.DIFF_ESTOP.init();
    manualControl = false;
    arm = new ArmCalculator();
    
    motorConfigDA.Slot0.kG *= -1;
    motorConfigDA.Slot1.kG *= -1;
    motorConfigDA.Slot2.kG *= -1;

    rotationRatio = DiffectorConfigs.rotationRatio;
    travelRatio = DiffectorConfigs.travelRatio;

    m_UA = new TalonFX(IDConstants.uaMotorID);
    m_DA = new TalonFX(IDConstants.daMotorID);
    io_Rotation = new CANcoder(IDConstants.armCANcoderID);
    io_Elevation = new AnalogPotentiometer(IDConstants.armPotID);
    
    m_UA.getConfigurator().apply(motorConfigUA);
    m_DA.getConfigurator().apply(motorConfigDA);
    
    elevation = Presets.startPosition.getX();

    positionOveride(getMeasuredElevation(), getMeasuredAngle());
    
    targetElevation = elevation;
    targetAngle     = angle;
    
    targetPosition  = new Translation2d(targetElevation, targetAngle);
    
    oldTarget       = targetPosition;
    relativeTarget  = targetPosition;
    
    motorTargets = calculateMotorTargets(targetPosition);

    motionMagicRequester = new MotionMagicVoltage(0);

    plannedPathPoints.clear();
    plannedPathPoints.add(targetPosition);

    SD.CALIBRATE_DIFF.init();
    SD.CALIBRATE_DIFF_TARGET.init();
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
    elevation = ((Units.rotationsToDegrees(m_UA.getPosition().getValueAsDouble()) - Units.rotationsToDegrees(m_DA.getPosition().getValueAsDouble())) / 2) * travelRatio;
    angle = ((Units.rotationsToDegrees(m_UA.getPosition().getValueAsDouble()) + Units.rotationsToDegrees(m_DA.getPosition().getValueAsDouble())) * rotationRatio) / 2;
    armPosition = new Translation2d(elevation, angle);
    
    if (Presets.lowDiffectorPositions.stream().anyMatch(relativeTarget::equals))
    {
      if (elevation < targetElevation - DiffectorGeometry.elevationTolerance)
        {eStop = true;}
    }
    else if 
    (
      (
        elevation < arm.checkPosition(armPosition) - DiffectorGeometry.elevationTolerance || 
        elevation > DiffectorGeometry.maxZ + (projectionElevation / 2)
      ) 
      && !manualControl
    )
    {eStop = true;}

    if (atPosition() && Presets.lowDiffectorPositions.stream().anyMatch(relativeTarget::equals))
    {
      calibrationCounter++;
      if (calibrationCounter == DiffectorConstants.calibrationDelay) 
      {
        SD.CALIBRATE_DIFF.put(true);
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
  public double getMeasuredAngle()
  { // Encoder outputs is geared 1:1 to the arm
    return Units.rotationsToDegrees(-io_Rotation.getPosition().getValueAsDouble());
  }

  /**
   * Arm Elevation as measured from potentiometer
   * @return Height of centre of rotation over ground, metres 
   * Will return the estimated value if the sensor reading is invalid
   */
  public double getMeasuredElevation()
  {
    if (io_Elevation.get() < DiffectorConstants.potErrValue)
      {return elevation;}
    return DiffectorConstants.potInterpolation.get(io_Elevation.get());
  }

  /**
   * Gets absolute arm rotation
   * @return Arm rotation, wrapped, degrees anticlockwise, 0 = coral at top, [0..360]
   */
  public double getRelativeRotation()
    {return Conversions.mod(angle, 360);}

  public Translation2d getRelativeTarget()
    {return relativeTarget;}

  private void calculatePath()
  {
    targetPosition = new Translation2d(targetElevation, targetAngle);
    relativeTarget = new Translation2d(targetElevation, Conversions.mod(targetAngle, 360));

    if (!targetPosition.equals(oldTarget))
    {
      if 
      (
        !(
          MathUtil.isNear(RobotContainer.swerveState.Pose.getX(), FieldUtils.fieldLength / 2, DiffectorGeometry.bargeSafetyWidth) &&
          targetPosition.getX() > DiffectorGeometry.bargeSafetyHeight
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
    if (RobotContainer.algae)
    { // Reduce speed when holding Algae
      m_UA.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicCruiseVelocity(DiffectorConfigs.diffectorAlgaeCruise));
      m_DA.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicCruiseVelocity(DiffectorConfigs.diffectorAlgaeCruise));
    }
    else
    {
      m_UA.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicCruiseVelocity(DiffectorConfigs.diffectorCruise));
      m_DA.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicCruiseVelocity(DiffectorConfigs.diffectorCruise));
    }

    if (MathUtil.isNear(angle, target.getY(), DiffectorGeometry.angleTolerance))
    { // If movement is only elevation, use elevation acceleration limits
      m_UA.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicAcceleration(DiffectorConfigs.diffectorElevationAcceleration));
      m_DA.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicAcceleration(DiffectorConfigs.diffectorElevationAcceleration));
    }
    else
    { // If movement includes rotation, use rotation acceleration limits
      if (RobotContainer.algae)
      {
        m_UA.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicAcceleration(DiffectorConfigs.diffectorAlgaeRotationAcceleration));
        m_DA.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicAcceleration(DiffectorConfigs.diffectorAlgaeRotationAcceleration));
      }
      else
      {
        m_UA.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicAcceleration(DiffectorConfigs.diffectorRotationAcceleration));
        m_DA.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicAcceleration(DiffectorConfigs.diffectorRotationAcceleration));
      }
    }

    double[] calculatedTargets = new double[2];

    calculatedTargets[0] = (target.getY() / rotationRatio) + (target.getX() / travelRatio);
    calculatedTargets[1] = (target.getY() / rotationRatio) - (target.getX() / travelRatio);

    return calculatedTargets;
  }

  /** Returns true if the diffector is at its current target angle */
  public boolean atAngle()
    {return Math.abs(angle - targetAngle) < DiffectorGeometry.angleTolerance;}

  /** Returns true if the diffector is at its current target elevation */
  public boolean atElevation()
    {return Math.abs(elevation - targetElevation) < DiffectorGeometry.elevationTolerance;}

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
      Math.abs(elevation - checkTarget.getX()) < DiffectorGeometry.elevationTolerance &&
      Math.abs(angle - checkTarget.getY()) < DiffectorGeometry.angleTolerance;
  }

  /**
   * Returns true if the diffector is at the given relative position
   * @param checkTarget target elevation/rotation to check against
   */
  public boolean atRelativePosition(Translation2d checkTarget)
  {
    return
      Math.abs(elevation - checkTarget.getX()) < DiffectorGeometry.elevationTolerance &&
      Math.abs(getRelativeRotation() - Conversions.mod(checkTarget.getY(), 360)) < DiffectorGeometry.angleTolerance;
  }

  /** Returns true if the diffector is safely in climb position */
  public boolean climbReady()
    {return atPosition(Presets.climbPosition);}

  /** Returns true if the diffector is safely above the path of the climber */
  public boolean climbSafe()
  {
    return
    (
      elevation >= DiffectorGeometry.climberClearanceThreshold &&
      (
        MathUtil.isNear(getRelativeRotation(),  90, DiffectorGeometry.angleTolerance) ||
        MathUtil.isNear(getRelativeRotation(), 270, DiffectorGeometry.angleTolerance)
      )
    );
  }

  /** 
   * Sets the Diffector arm to unwind to starting position 
   * @return Safe to stow
   */
  public boolean unwind()
  {
    manualControl = false;
    targetAngle = Presets.startPosition.getY();
    targetElevation = DiffectorGeometry.safeElevation;
    return Math.abs(angle) < stowThreshold;
  }

  public void setElevationTarget(double newTarget)
  {
    manualControl = false;
    targetElevation = MathUtil.clamp(newTarget, DiffectorGeometry.minZ, DiffectorGeometry.maxZ);
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
    boolean nearSetPos = MathUtil.isNear(elevation, setElevation, DiffectorGeometry.elevationTolerance) && MathUtil.isNear(angle, setAngle, DiffectorGeometry.angleTolerance);

    m_UA.setPosition(Units.degreesToRotations((setAngle / rotationRatio) + (setElevation / travelRatio)));
    m_DA.setPosition(Units.degreesToRotations((setAngle / rotationRatio) - (setElevation / travelRatio)));

    elevation = setElevation;
    angle = setAngle;

    return nearSetPos;
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
    .onlyIf(() -> !RobotState.isDisabled() || SD.OVERRIDE.get())
    .ignoringDisable(true);
  }

  public Command moveAndWaitCommand(Translation2d targetPosition)
  {
    return moveToCommand(targetPosition).andThen(Commands.waitUntil(this::atPosition));
  }

  public Command stationIntakePosCommand(Supplier<Translation2d> robotPos, BooleanSupplier algae)
  {
    if (robotPos.get().getX() > FieldUtils.fieldLength/2 ^ robotPos.get().getY() > FieldUtils.fieldWidth/2)
    {
      if (algae.getAsBoolean())
        return moveToCommand(Presets.coralClawStbdPosition);
      else
        return moveToCommand(Presets.coralIntakeStbdPosition);  
    }
    else
    {
      if (algae.getAsBoolean())
        return moveToCommand(Presets.coralClawPortPosition);
      else
        return moveToCommand(Presets.coralIntakePortPosition);  
    }
  }

  public Command algaeIntakePosCommand(Supplier<Translation2d> robotPos, boolean level2)
  {
    int nearestReefFace = FieldUtils.getNearestReefFace(robotPos.get());
    boolean portReefFace = (nearestReefFace == 5 || nearestReefFace == 6);

    Translation2d target = 
    level2 
    ?
    portReefFace ? Presets.algae2PortPosition : Presets.algae2StbdPosition
    :
    portReefFace ? Presets.algae3PortPosition : Presets.algae3StbdPosition;

    return moveAndWaitCommand(target);
  }

  public Command algaeIntakePosCommand(int nearestReefFace)
  {
    return defer(() -> algaeIntakePosCommand(() -> RobotContainer.swerveState.Pose.getTranslation(), nearestReefFace % 2 == 0));
  }

  public Command algaeIntakePosCommand(boolean level2)
  {
    Supplier<Translation2d> robotPos = () -> RobotContainer.swerveState.Pose.getTranslation();

    return defer(() -> algaeIntakePosCommand(robotPos, level2));
  }

  public Command coralScorePosInstantCommand(Supplier<Translation2d> robotPos, int level)
  {
    int nearestReefFace = FieldUtils.getNearestReefFace(robotPos.get());
    boolean portReefFace = (nearestReefFace == 5 || nearestReefFace == 6);

    Translation2d target = 
    switch (level)
    {
      case 4 -> portReefFace ? Presets.coral4PortPosition : Presets.coral4StbdPosition;

      case 3 -> portReefFace ? Presets.coral3PortPosition : Presets.coral3StbdPosition;

      case 2 -> portReefFace ? Presets.coral2PortPosition : Presets.coral2StbdPosition;

      case 1 -> portReefFace ? Presets.coral1ClawPortPosition : Presets.coral1ClawStbdPosition;

      case 0 -> portReefFace ? Presets.coral1PortPosition : Presets.coral1StbdPosition;

      default -> Presets.coralStowPosition;
    };

    return moveToCommand(target);
  }

  public Command coralScorePosCommandUndeferred(Supplier<Translation2d> robotPos, int level)
  {
    return coralScorePosInstantCommand(robotPos, level).andThen(Commands.waitUntil(this::atPosition));
  }

  public Command coralScorePosCommand(int level)
  {
    return defer(() -> coralScorePosCommandUndeferred(() -> RobotContainer.swerveState.Pose.getTranslation(), level));
  }

  @Override
  public void periodic() 
  { 
    if (SD.CALIBRATE_DIFF.get())
    {
      positionOveride(getMeasuredElevation(), getMeasuredAngle());
      SD.CALIBRATE_DIFF.put(false);
    }
    
    if (SD.OVERRIDE.get())
    {
      if (SD.CALIBRATE_DIFF_TARGET.get())
      {
        positionOveride(targetElevation, targetAngle);
        plannedPathPoints.clear();
        SD.CALIBRATE_DIFF_TARGET.put(false);
      }
    }

    calculatePosition();
    
    if 
    (
      Math.abs(m_UA.getTorqueCurrent().getValueAsDouble()) > DiffectorConfigs.motorStallCurrent ||
      Math.abs(m_DA.getTorqueCurrent().getValueAsDouble()) > DiffectorConfigs.motorStallCurrent
    )
    {
      eStop = true;
      SD.DIFF_ESTOP.put(true);
    }
    else
      {eStop = SD.DIFF_ESTOP.get();}
    
    if (eStop)
    {
      m_UA.set(0);
      m_DA.set(0);
      eStop = SD.DIFF_ESTOP.get();
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
        m_UA.setVoltage((manualRotation * Constants.Control.manualDiffectorRotationScalar) + (manualElevation * Constants.Control.manualDiffectorElevationScalar));
        m_DA.setVoltage((manualRotation * Constants.Control.manualDiffectorRotationScalar) - (manualElevation * Constants.Control.manualDiffectorElevationScalar));
      }
      else
      {
        calculatePath();

        m_UA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[0])));//.withSlot(getSlot()));
        m_DA.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[1])));//.withSlot(getSlot()));
      }
    }
    SmartDashboard.putNumber("ua current", Math.abs(m_UA.getTorqueCurrent().getValueAsDouble()));
    SmartDashboard.putNumber("ua current", Math.abs(m_DA.getTorqueCurrent().getValueAsDouble()));
    SmartDashboard.putNumber("ua Speed", Math.abs(m_UA.getRotorVelocity().getValueAsDouble()));
    SmartDashboard.putNumber("da Speed", Math.abs(m_DA.getRotorVelocity().getValueAsDouble()));
    SD.DIFF_ELEVATION_TARGET.put(targetElevation);
    SD.DIFF_ANGLE_TARGET.put(targetAngle);
    SD.DIFF_ELEVATION.put(elevation);
    SD.DIFF_ANGLE.put(angle);

    SD.DIFF_UA_ER.put(motorTargets[0] - Units.rotationsToDegrees(m_UA.getPosition().getValueAsDouble()));
    SD.DIFF_DA_ER.put(motorTargets[1] - Units.rotationsToDegrees(m_DA.getPosition().getValueAsDouble()));

    SD.DIFF_HEIGHT.put(elevation - arm.checkAngle(angle));
    SD.SENSOR_DIFF_ANGLE.put(getMeasuredAngle());
    SD.DIFF_ANGLE_ER.put(angle - getMeasuredAngle());

    SD.SENSOR_DIFF_POT.put(io_Elevation.get());
  }
}
