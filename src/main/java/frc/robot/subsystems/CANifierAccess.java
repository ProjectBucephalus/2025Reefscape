// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;

public class CANifierAccess extends SubsystemBase 
{
  CANifier io_CANifier = new CANifier(IDConstants.canifierID);

  public CANifierAccess() 
  {

  }

  public boolean algaeManiSensor()
    {return io_CANifier.getGeneralInput(IDConstants.algaeManipulatorDIO);}

  public boolean coralManiStbdSensor()
    {return io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOStbd);}

  public boolean coralManiPortSensor()
    {return io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOPort);}

  @Override
  public void periodic() 
  {

  }
}

// io_CANifier.getGeneralInput(GeneralPin.SPI_CLK_PWM0P);  PWM1, PWM2, PWM0
