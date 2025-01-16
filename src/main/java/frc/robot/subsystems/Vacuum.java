// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Vacuum extends SubsystemBase 
{
  private Solenoid vacuumSolenoid;
  private VictorSPX m_vacuum1;
  private VictorSPX m_vacuum2;
  private PowerDistribution pdh;

  /** Flag for whether vacuum pump motors are on or off */
  private boolean vacuumControlFlag;

  /** Creates a new Vacuum. */
  public Vacuum() 
  {
    vacuumSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Vacuum.vacuumSolenoidID);
    m_vacuum1 = new VictorSPX(Constants.Vacuum.vacuumMotor1ID);
    m_vacuum2 = new VictorSPX(Constants.Vacuum.vacuumMotor2ID);
    pdh = new PowerDistribution(Constants.Vacuum.pdhID, ModuleType.kRev);

    vacuumControlFlag = false;
  }
  
  /** Activate vacuum solenoid. Directionality TBD */
  public void solenoidOn()
  {
    vacuumSolenoid.set(true);
  }

  /** Deactivate vacuum solenoid. Directionality TBD */
  public void solenoidOff()
  {
    vacuumSolenoid.set(false);
  }

  /** Enable vacuum pump motors, generating vacuum */
  public void pumpOn()
  {
    vacuumControlFlag = true;
    m_vacuum1.set(VictorSPXControlMode.PercentOutput, 1);
    m_vacuum2.set(VictorSPXControlMode.PercentOutput, 1);
  }

  /** Sets vacuum pump motors to a low power, to maintain a vacuum if it's leaking */
  public void pumpOnLow()
  {
    m_vacuum1.set(VictorSPXControlMode.PercentOutput, Constants.Vacuum.vacuumLowMotorPercent);
    m_vacuum2.set(VictorSPXControlMode.PercentOutput, Constants.Vacuum.vacuumLowMotorPercent);
  }
  
  /** Disable vacuum pump motors, generating vacuum */
  public void pumpOff()
  {
    vacuumControlFlag = false;
    m_vacuum1.set(VictorSPXControlMode.PercentOutput, 0);
    m_vacuum2.set(VictorSPXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() 
  { 
    /* 
     * If either vacuum motors are drawing over a certain amount of current, and the vacuum pump is set to be on, disables pump. 
     * This is to turn off the motors once a vacuum is generated 
     */
    if 
    (
      (pdh.getCurrent(Constants.Vacuum.vacuumMotor1PDHChannel) > Constants.Vacuum.vacuumMotorMaxCurrent 
      || pdh.getCurrent(Constants.Vacuum.vacuumMotor2PDHChannel) > Constants.Vacuum.vacuumMotorMaxCurrent)
      && vacuumControlFlag
    )
    {
      pumpOff();
    }
  }
}
