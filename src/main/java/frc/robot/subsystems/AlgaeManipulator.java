// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class AlgaeManipulator extends SubsystemBase {

  private TalonFX algaeMotor;

  private DigitalInput beamBreak1;

  public enum algaeManipulatorStatus
  {
    INTAKE,
    HOLDING,
    NET,
    PROCESSOR,
    EMPTY
  }

  public AlgaeManipulator() 
  {
    algaeMotor = new TalonFX(0);
    beamBreak1 = new DigitalInput(0);
  }

  private void setAlgaeIntakeSpeed(double Speed)
  {
    algaeMotor.set(Speed);
  }

  private void getBeamBreak1State()
  {
    beamBreak1.get();
  }
  
  public void setAlgaeManipulatorStatus(algaeManipulatorStatus status)
  {
    switch(status)
    {
      case INTAKE:
      setAlgaeIntakeSpeed(0);
      getBeamBreak1State();
      break;

      case HOLDING:
      setAlgaeIntakeSpeed(0);
      getBeamBreak1State();
      break;

      case NET:
      setAlgaeIntakeSpeed(0);
      getBeamBreak1State();
      break;

      case PROCESSOR:
      setAlgaeIntakeSpeed(0);
      getBeamBreak1State();
      break;

      case EMPTY:
      setAlgaeIntakeSpeed(0);
      getBeamBreak1State();
      break;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
