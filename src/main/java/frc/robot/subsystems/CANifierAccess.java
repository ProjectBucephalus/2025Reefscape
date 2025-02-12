// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;

public class CANifierAccess extends SubsystemBase 
{
  CANifier io_CANifier = new CANifier(23);

  public CANifierAccess() 
  {

  }

  public boolean algaeManiCANifier()
    {return io_CANifier.getGeneralInput(GeneralPin.SPI_MISO_PWM2P);}

  public boolean coralMani1CANifier()
    {return io_CANifier.getGeneralInput(GeneralPin.SPI_MOSI_PWM1P);}

  public boolean coralMani2CANifier()
    {return io_CANifier.getGeneralInput(GeneralPin.SPI_CLK_PWM0P);}

  @Override
  public void periodic() 
  {

  }
}

// io_CANifier.getGeneralInput(GeneralPin.SPI_CLK_PWM0P);  PWM1, PWM2, PWM0
