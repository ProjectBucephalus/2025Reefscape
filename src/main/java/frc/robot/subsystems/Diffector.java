// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;

public class Diffector extends SubsystemBase 
{
  public enum CargoStates{empty, oneItem, twoItem}
  /* Name is effect of motor when running clockwise/positive (e.g. elevator Up, arm Clockwise) */
  /** motor L in kirby's docs */
  private static TalonFX m_diffectorUC;
  /** motor R in kirby's docs */
  private static TalonFX m_diffectorDC;
  private final MotionMagicVoltage motionMagicRequester;
  private double targetElevation;
  private double targetAngle;
  private static double rotationRatio;
  private static double travelRatio;
  private double[] motorTargets = new double[2];
  private TalonFXConfiguration motorConfig;
  private CargoStates cargoState;
  private boolean coral;
  private boolean algae;
  private double maxAbsPos = Constants.Diffector.maxAbsPos;
  private double turnBackThreshold = Constants.Diffector.turnBackThreshold;
  private double stowThreshold = Constants.Diffector.angleTolerance;
  private double offset;
  private double altOffset;
  private double armPos;
  private DutyCycleEncoder encoder = new DutyCycleEncoder(6);
  
  
  /** Creates a new Diffector. */
  public Diffector() 
  {
    motorConfig = CTREConfigs.diffectorFXConfig;

    m_diffectorUC = new TalonFX(Constants.Diffector.ucMotorID);
    m_diffectorDC = new TalonFX(Constants.Diffector.uaMotorID);

    targetElevation = 0;
    targetAngle = 0;

    m_diffectorUC.getConfigurator().apply(motorConfig);
    m_diffectorDC.getConfigurator().apply(motorConfig);

    rotationRatio = Constants.Diffector.rotationRatio;
    travelRatio = Constants.Diffector.travelRatio;

    cargoState = Constants.Diffector.startingCargoState;

    motionMagicRequester = new MotionMagicVoltage(0);
  }

  /**
   * Calculates arm rotation based on motor positions
   * @return Arm rotation, degrees clockwise, 0 = coral at top
   */
  public static double getArmPos()
  {
    return (m_diffectorUC.getPosition().getValueAsDouble() + m_diffectorDC.getPosition().getValueAsDouble()) * rotationRatio;
  }

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
  public static double getElevatorPos()
  {
    return ((m_diffectorUC.getPosition().getValueAsDouble() - m_diffectorDC.getPosition().getValueAsDouble()) * travelRatio);
  }

  /**
   * Calculates the position to drive each motor to, based on the target positions for the elevator and arm
   * @param elevatorTarget
   * @param armTarget
   * @return
   */
  public double[] calculateMotorTargets(double elevatorTarget, double armTarget)
  {
    double[] calculatedTargets = new double[2];

    calculatedTargets[0] = (armTarget / rotationRatio) + (elevatorTarget / travelRatio);
    calculatedTargets[1] = (armTarget / rotationRatio) - (elevatorTarget / travelRatio);

    return calculatedTargets;
  }

  private void calculateMotorTargets()
  {
    motorTargets = calculateMotorTargets(targetElevation, targetAngle);
  }

  public boolean armAtAngle()
  {
    return Math.abs(getArmPos() - targetAngle) < Constants.Diffector.angleTolerance;
  }

  public boolean elevatorAtElevation()
  {
    return Math.abs(getElevatorPos() - targetElevation) < Constants.Diffector.elevationTolerance;
  }

  /** 
   * Sets the Diffector arm to unwind to starting position 
   * @return Safe to stow
   */
  public boolean unwind()
  {
      targetAngle = Constants.Diffector.returnPos;
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
  public void goAntiClockwise(double targetAngle)
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
          altOffset = offset - Math.copySign(360, offset);

          if (Math.abs(armPos + offset) > Math.abs(armPos + altOffset))
              {targetAngle = (armPos + altOffset);}
          
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
  {
    return targetAngle;
  }

  public void setElevatorTarget(double newTarget)
  {
    targetElevation = newTarget;
  }

  public double getElevatorTarget()
  {
    return targetElevation;
  }

  public void setCargoState(CargoStates newCargoState)
  {
    cargoState = newCargoState;
  }

  public CargoStates getCargoState()
  {
    return cargoState;
  }

  public void setCoral(boolean coralState)
  {
    coral = coralState;
    updateCargoState();
  }

  public void setAlgae(boolean algaeState)
  {
    algae = algaeState;
    updateCargoState();
  }

  private void updateCargoState()
  {
    if(coral && algae)
    {
      setCargoState(CargoStates.twoItem);
    }
    else if(coral ^ algae)
    {
      setCargoState(CargoStates.oneItem);
    }
    else if(!coral && !algae)
    {
      setCargoState(CargoStates.empty);
    }
  }

  private int getSlot()
  {
    switch (cargoState) 
    {
      case empty: return 0;
      case oneItem: return 1;
      case twoItem: return 2;
      default: return 0;
    }
  }

  @Override
  public void periodic() 
  { 
    calculateMotorTargets();

    m_diffectorUC.setControl(motionMagicRequester.withPosition(motorTargets[0]).withSlot(getSlot()));
    m_diffectorDC.setControl(motionMagicRequester.withPosition(motorTargets[1]).withSlot(getSlot()));

    armPos = getArmPos();

    SmartDashboard.putNumber("Elevator Height", getElevatorPos());
    SmartDashboard.putNumber("Arm Rotation", getArmPos());
  }
}
