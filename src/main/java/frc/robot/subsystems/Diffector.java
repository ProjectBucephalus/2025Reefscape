// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;

public class Diffector extends SubsystemBase 
{
  /* Name is effect of motor when running forwards (e.g. elevator Up, arm Clockwise) */
  private static TalonFX m_diffectorUC;
  private static TalonFX m_diffectorUA;
  private final MotionMagicVoltage motionMagicRequester;
  private double targetElevator;
  private double targetArm;
  /** Distance travelled by elevator in mm, from starting point of 0 mm (all the way down) */
  private double elevatorTravel;
    /** Distance travelled by arm in degrees, from starting point of 0 degrees ( ) */
    private double armTravel;

  /** Creates a new Diffector. */
  public Diffector() 
  {
    m_diffectorUC = new TalonFX(Constants.DiffectorConstants.ucMotorID);
    m_diffectorUA = new TalonFX(Constants.DiffectorConstants.uaMotorID);

    targetElevator = 0;
    targetArm = 0;

    m_diffectorUC.getConfigurator().apply(CTREConfigs.diffectorFXConfig);
    m_diffectorUA.getConfigurator().apply(CTREConfigs.diffectorFXConfig);

    motionMagicRequester = new MotionMagicVoltage(0);
  }

  /**
   * Calculates arm rotation based on motor positions
   * @return Arm rotation in degrees
   */
  private double getArmPos()
  {
    /* PLACEHOLDER confirm once kirby sends through formulas */
    return ((m_diffectorUC.getPosition().getValueAsDouble() + m_diffectorUA.getPosition().getValueAsDouble()) / 2) * Constants.DiffectorConstants.rotationRatio;
  }

    /**
   * Calculates elevator height based on motor positions
   * @return Elevator height in mm
   */
  private double getElevatorPos()
  {
    /* PLACEHOLDER replace once kirby sends through formulas */
    return Constants.DiffectorConstants.travelRatio;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
