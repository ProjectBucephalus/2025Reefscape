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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DiffectorConstants;
import frc.robot.constants.Constants.DiffectorConstants.IKGeometry;
import frc.robot.constants.Constants.DiffectorConstants.Presets;
import frc.robot.constants.IDConstants;
import frc.robot.util.ArmCalculator;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;
import frc.robot.util.SD;

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
  private final TalonFXConfiguration motorConfigUA = new TalonFXConfiguration()
  {{
    /* Diffector Motor Gneral Config */
    MotorOutput.NeutralMode = Constants.DiffectorConstants.neutralMode;
    Feedback.SensorToMechanismRatio = Constants.DiffectorConstants.gearboxRatio;

    /* Diffector Motor Config (Default) */
    Slot0.kG = Constants.DiffectorConstants.diffectorMotorKG;
    Slot0.kS = Constants.DiffectorConstants.diffectorMotorKS;
    Slot0.kV = Constants.DiffectorConstants.diffectorMotorKV;
    Slot0.kP = Constants.DiffectorConstants.diffectorMotorKP;
    Slot0.kI = Constants.DiffectorConstants.diffectorMotorKI;
    Slot0.kD = Constants.DiffectorConstants.diffectorMotorKD;
    
    /* Diffector Motor Config (Virtual Spring) */
    Slot1.kG = Constants.DiffectorConstants.diffectorMotorKGSpring;
    Slot1.kS = Constants.DiffectorConstants.diffectorMotorKSSpring;
    Slot1.kV = Constants.DiffectorConstants.diffectorMotorKVSpring;
    Slot1.kP = Constants.DiffectorConstants.diffectorMotorKPSpring;
    Slot1.kI = Constants.DiffectorConstants.diffectorMotorKISpring;
    Slot1.kD = Constants.DiffectorConstants.diffectorMotorKDSpring;

    /* Diffector MotionMagic Default Config */
    MotionMagic.MotionMagicCruiseVelocity = Constants.DiffectorConstants.diffectorCruise;
    MotionMagic.MotionMagicAcceleration = Constants.DiffectorConstants.diffectorRotationAcceleration;
  }};
  private final TalonFXConfiguration motorConfigDA = motorConfigUA;
  private final double stowThreshold = Constants.DiffectorConstants.angleTolerance;
  
  /* Name is effect of motor when running anticlockwise/positive (e.g. elevator Up, arm Anticlockwise) */
  /** starboard-side motor(?), forward direction drives carriage up and anticlockwise */
  private static TalonFX uaMotor;
  /** port-side motor(?), forward direction drives carriage down and anticlockwise */
  private static TalonFX daMotor;
  private CANcoder encoder;
  private AnalogPotentiometer potentiometer;

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
    SD.DIFF_ESTOP.init();
    manualControl = false;
    arm = new ArmCalculator();
    
    motorConfigDA.Slot0.kG = -motorConfigUA.Slot0.kG;
    motorConfigDA.Slot1.kG = -motorConfigUA.Slot1.kG;
    motorConfigDA.Slot2.kG = -motorConfigUA.Slot2.kG;

    rotationRatio = Constants.DiffectorConstants.rotationRatio;
    travelRatio = Constants.DiffectorConstants.travelRatio;

    uaMotor = new TalonFX(IDConstants.uaMotorID);
    daMotor = new TalonFX(IDConstants.daMotorID);
    encoder = new CANcoder(IDConstants.armCANcoderID);
    potentiometer = new AnalogPotentiometer(IDConstants.armPotID);

    targetPosition  = Presets.startPosition;

    targetElevation = targetPosition.getX();
    targetAngle     = targetPosition.getY();
    oldTarget       = targetPosition;
    relativeTarget  = targetPosition;

    uaMotor.getConfigurator().apply(motorConfigUA);
    daMotor.getConfigurator().apply(motorConfigDA);
    
    motorTargets = calculateMotorTargets(targetPosition);

    if (Conversions.mod(getMeasuredAngle(), 360) > Constants.DiffectorConstants.angleTolerance && Conversions.mod(getMeasuredAngle(), 360) < 360 - Constants.DiffectorConstants.angleTolerance) 
    {
      eStop = true;
    }

    elevation = targetElevation;
    positionOveride(getMeasuredElevation(), getMeasuredAngle());

    calculatePosition();

    updateSpringState();

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
    elevation = ((Units.rotationsToDegrees(uaMotor.getPosition().getValueAsDouble()) - Units.rotationsToDegrees(daMotor.getPosition().getValueAsDouble())) / 2) * travelRatio;
    angle = ((Units.rotationsToDegrees(uaMotor.getPosition().getValueAsDouble()) + Units.rotationsToDegrees(daMotor.getPosition().getValueAsDouble())) * rotationRatio) / 2;
    armPosition = new Translation2d(elevation, angle);
    
    if (Presets.lowDiffectorPositions.stream().anyMatch(position -> relativeTarget.equals(position)))
    {
      if (elevation < targetElevation - DiffectorConstants.elevationTolerance)
        {eStop = true;}
    }
    else if 
    (
      (
        elevation < arm.checkPosition(armPosition) - DiffectorConstants.elevationTolerance || 
        elevation > DiffectorConstants.maxZ + projectionElevation
      ) 
      && !manualControl
    )
    {eStop = true;}

    if (atPosition())
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
    return Units.rotationsToDegrees(encoder.getPosition().getValueAsDouble());
  }

  /**
   * Arm Elevation as measured from potentiometer
   * @return Height of centre of rotation over ground, metres 
   * Will return the estimated value if the sensor reading is invalid
   */
  public double getMeasuredElevation()
  {
    if (potentiometer.get() < DiffectorConstants.potErrValue)
      {return elevation;}
    return MathUtil.interpolate(DiffectorConstants.potMin, DiffectorConstants.potMax, potentiometer.get());
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
    if (RobotContainer.algae)
    { // Reduce speed when holding Algae
      uaMotor.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicCruiseVelocity(DiffectorConstants.diffectorAlgaeCruise));
      daMotor.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicCruiseVelocity(DiffectorConstants.diffectorAlgaeCruise));
    }
    else
    {
      uaMotor.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicCruiseVelocity(DiffectorConstants.diffectorCruise));
      daMotor.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicCruiseVelocity(DiffectorConstants.diffectorCruise));
    }

    if (MathUtil.isNear(angle, target.getY(), DiffectorConstants.angleTolerance))
    { // If movement is only elevation, use elevation acceleration limits
      uaMotor.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicAcceleration(DiffectorConstants.diffectorElevationAcceleration));
      daMotor.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicAcceleration(DiffectorConstants.diffectorElevationAcceleration));
    }
    else
    { // If movement includes rotation, use rotation acceleration limits
      if (RobotContainer.algae)
      {
        uaMotor.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicAcceleration(DiffectorConstants.diffectorAlgaeRotationAcceleration));
        daMotor.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicAcceleration(DiffectorConstants.diffectorAlgaeRotationAcceleration));
      }
      else
      {
        uaMotor.getConfigurator().apply(motorConfigUA.MotionMagic.withMotionMagicAcceleration(DiffectorConstants.diffectorRotationAcceleration));
        daMotor.getConfigurator().apply(motorConfigDA.MotionMagic.withMotionMagicAcceleration(DiffectorConstants.diffectorRotationAcceleration));
      }
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
    {return atPosition(Presets.climbPosition);}

  /** Returns true if the diffector is safely above the path of the climber */
  public boolean climbSafe()
  {
    return
    (
      elevation >= DiffectorConstants.climberClearanceThreshold &&
      (
        MathUtil.isNear(getRelativeRotation(),  90, DiffectorConstants.angleTolerance) ||
        MathUtil.isNear(getRelativeRotation(), 270, DiffectorConstants.angleTolerance)
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
    return Math.abs(angle) < stowThreshold;
  }

  public void setElevationTarget(double newTarget)
  {
    manualControl = false;
    targetElevation = MathUtil.clamp(newTarget, DiffectorConstants.minZ, DiffectorConstants.maxZ);
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
    setElevation = MathUtil.clamp(setElevation, DiffectorConstants.minZ, DiffectorConstants.maxZ);
    uaMotor.setPosition(Units.degreesToRotations((setAngle / rotationRatio) + (setElevation / travelRatio)));
    daMotor.setPosition(Units.degreesToRotations((setAngle / rotationRatio) - (setElevation / travelRatio)));

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
    .onlyIf(() -> !RobotState.isDisabled() || SD.OVERRIDE.get())
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
    portReefFace ? Presets.algae2PortPosition : Presets.algae2StbdPosition
    :
    portReefFace ? Presets.algae3PortPosition : Presets.algae3StbdPosition;

    return moveAndWaitCommand(target);
  }

  public Command algaeIntakePosCommand(int nearestReefFace)
  {
    return defer(() -> algaeIntakePosCommand(RobotContainer.swerveState.Pose.getTranslation(), nearestReefFace % 2 == 0, nearestReefFace));
  }

  public Command algaeIntakePosCommand(boolean level2)
  {
    Translation2d robotPos = RobotContainer.swerveState.Pose.getTranslation();

    return defer(() -> algaeIntakePosCommand(robotPos, level2, FieldUtils.getNearestReefFace(robotPos)));
  }

  public Command coralScorePosInstantCommand(Translation2d robotPos, int level)
  {
    int nearestReefFace = FieldUtils.getNearestReefFace(robotPos);
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

  public Command coralScorePosCommand(Translation2d robotPos, int level)
  {
    return defer(() -> coralScorePosInstantCommand(robotPos, level).andThen(Commands.waitUntil(() -> atPosition())));
  }

  public Command coralScorePosCommand(int level)
  {
    return coralScorePosCommand(RobotContainer.swerveState.Pose.getTranslation(), level);
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
    updateSpringState();
    
    if 
    (
      Math.abs(uaMotor.getTorqueCurrent().getValueAsDouble()) > Constants.DiffectorConstants.motorStallCurrent ||
      Math.abs(daMotor.getTorqueCurrent().getValueAsDouble()) > Constants.DiffectorConstants.motorStallCurrent
    )
    {
      eStop = true;
      SD.DIFF_ESTOP.put(true);
    }
    else
      {eStop = SD.DIFF_ESTOP.get();}
    
    if (eStop)
    {
      uaMotor.set(0);
      daMotor.set(0);
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
        uaMotor.setVoltage((manualRotation * Constants.Control.manualDiffectorRotationScalar) + (manualElevation * Constants.Control.manualDiffectorElevationScalar));
        daMotor.setVoltage((manualRotation * Constants.Control.manualDiffectorRotationScalar) - (manualElevation * Constants.Control.manualDiffectorElevationScalar));
      }
      else
      {
        calculatePath();

        uaMotor.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[0])));//.withSlot(getSlot()));
        daMotor.setControl(motionMagicRequester.withPosition(Units.degreesToRotations(motorTargets[1])));//.withSlot(getSlot()));
      }
    }
    SD.DIFF_ELEVATION_TARGET.put(targetElevation);
    SD.DIFF_ANGLE_TARGET.put(targetAngle);
    SD.DIFF_ELEVATION.put(elevation);
    SD.DIFF_ANGLE.put(angle);

    SD.DIFF_UA_ER.put(motorTargets[0] - Units.rotationsToDegrees(uaMotor.getPosition().getValueAsDouble()));
    SD.DIFF_DA_ER.put(motorTargets[1] - Units.rotationsToDegrees(daMotor.getPosition().getValueAsDouble()));

    SD.DIFF_HEIGHT.put(elevation - arm.checkAngle(angle));
    SD.SENSOR_DIFF_ANGLE.put(getMeasuredAngle());
    SD.DIFF_ANGLE_ER.put(angle - getMeasuredAngle());

    SD.SENSOR_DIFF_POT.put(potentiometer.get());
  }
}
