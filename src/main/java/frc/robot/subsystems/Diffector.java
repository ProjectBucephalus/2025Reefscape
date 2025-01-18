// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;

public class Diffector extends SubsystemBase 
{
  /* Name is effect of motor when running clockwise/positive (e.g. elevator Up, arm Clockwise) */
  /** motor L in kirby's docs */
  private static TalonFX m_diffectorUC;
  /** motor R in kirby's docs */
  private static TalonFX m_diffectorDC;
  private final MotionMagicVoltage motionMagicRequester;
  private double targetElevator;
  private double targetArm;
  private final double rotationRatio;
  private final double travelRatio;
  private double[] motorTargets = new double[2];

  /** Creates a new Diffector. */
  public Diffector() 
  {
    m_diffectorUC = new TalonFX(Constants.DiffectorConstants.ucMotorID);
    m_diffectorDC = new TalonFX(Constants.DiffectorConstants.uaMotorID);

    targetElevator = 0;
    targetArm = 0;

    m_diffectorUC.getConfigurator().apply(CTREConfigs.diffectorFXConfig);
    m_diffectorDC.getConfigurator().apply(CTREConfigs.diffectorFXConfig);

    rotationRatio = Constants.DiffectorConstants.rotationRatio;
    travelRatio = Constants.DiffectorConstants.travelRatio;

    motionMagicRequester = new MotionMagicVoltage(0);
  }

  /**
   * Calculates arm rotation based on motor positions
   * @return Arm rotation in degrees
   */
  public double getArmPos()
  {
    return (m_diffectorUC.getPosition().getValueAsDouble() + m_diffectorDC.getPosition().getValueAsDouble()) * rotationRatio;
  }

  /**
   * Calculates elevator height based on motor positions
   * @return Elevator height in mm
   */
  public double getElevatorPos()
  {
    return ((m_diffectorUC.getPosition().getValueAsDouble() - m_diffectorDC.getPosition().getValueAsDouble())) * travelRatio;
  }

  private void calculateMotorTargets(double elevatorTarget, double armTarget)
  {
    motorTargets[0] = (armTarget / rotationRatio) + (elevatorTarget / travelRatio);
    motorTargets[1] = (armTarget / rotationRatio) - (elevatorTarget / travelRatio);
  }

  public void setArmTarget(double newTarget)
  {
    targetArm = newTarget;
  }

  public double getArmTarget()
  {
    return targetArm;
  }

  public void setElevatorTarget(double newTarget)
  {
    targetElevator = newTarget;
  }

  public double getElevatorTarget()
  {
    return targetElevator;
  }

  @Override
  public void periodic() 
  { 
    calculateMotorTargets(targetElevator, targetArm);
    m_diffectorUC.setControl(motionMagicRequester.withPosition(motorTargets[0]));
    m_diffectorDC.setControl(motionMagicRequester.withPosition(motorTargets[1]));

    SmartDashboard.putNumber("Elevator Height", getElevatorPos());
    SmartDashboard.putNumber("Arm Rotation", getArmPos());
  }
}
