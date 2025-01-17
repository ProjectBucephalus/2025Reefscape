// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class CoralManipulator extends SubsystemBase {

  private TalonFX coralMotor;

  private DigitalInput beamBreak1;
  private DigitalInput beamBreak2;

  private boolean useBeamBreak1 = false;
  private boolean useBeamBreak2 = false;


  public enum coralManipulatorStatus
  {
    INTAKE,
    DELIVERY,
    HOLDING
  }

  public CoralManipulator() 
  {
    coralMotor = new TalonFX(0);
    beamBreak1 = new DigitalInput(0);
    beamBreak2 = new DigitalInput(0);
  }

  private void setCoralIntakeSpeed(double Speed)
  {
    coralMotor.set(Speed);
  }

  private void getBeamBreak1State()
  {
    beamBreak1.get();
  }

  private void getBeamBreak2State()
  {
    beamBreak2.get();
  }

  public void setCoralManipulatorStatus(coralManipulatorStatus Status)
  {
    switch(Status)
    {
      case INTAKE:
      setCoralIntakeSpeed(0);
      break;

      case DELIVERY:
      setCoralIntakeSpeed(0);
      break;

      case HOLDING:
      setCoralIntakeSpeed(0);
      break;
    }
  }
  

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
