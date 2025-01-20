// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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
  private double targetElevator;
  private double targetArm;
  private static double rotationRatio;
  private static double travelRatio;
  private double[] motorTargets = new double[2];
  private TalonFXConfiguration motorConfig;
  private CargoStates cargoState;
  private boolean coral;
  private boolean algae;


  /** Creates a new Diffector. */
  public Diffector() 
  {
    motorConfig = CTREConfigs.diffectorFXConfig;

    m_diffectorUC = new TalonFX(Constants.Diffector.ucMotorID);
    m_diffectorDC = new TalonFX(Constants.Diffector.uaMotorID);

    targetElevator = 0;
    targetArm = 0;

    m_diffectorUC.getConfigurator().apply(motorConfig);
    m_diffectorDC.getConfigurator().apply(motorConfig);

    rotationRatio = Constants.Diffector.rotationRatio;
    travelRatio = Constants.Diffector.travelRatio;

    cargoState = Constants.Diffector.startingCargoState;

    motionMagicRequester = new MotionMagicVoltage(0);
  }

  /**
   * Calculates arm rotation based on motor positions
   * @return Arm rotation in degrees
   */
  public static double getArmPos()
  {
    return (m_diffectorUC.getPosition().getValueAsDouble() + m_diffectorDC.getPosition().getValueAsDouble()) * rotationRatio;
  }

  /**
   * Calculates elevator height based on motor positions
   * @return Elevator height in mm
   */
  public static double getElevatorPos()
  {
    return (m_diffectorUC.getPosition().getValueAsDouble() - m_diffectorDC.getPosition().getValueAsDouble()) * travelRatio;
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
    motorTargets = calculateMotorTargets(targetElevator, targetArm);
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

    SmartDashboard.putNumber("Elevator Height", getElevatorPos());
    SmartDashboard.putNumber("Arm Rotation", getArmPos());
  }
}
