// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.constants.IDConstants;
import frc.robot.util.SD;
import frc.robot.util.SD.Key;

import com.ctre.phoenix.CANifier;

public class CANifierAccess
{
  CANifier io_CANifier = new CANifier(IDConstants.canifierID);

  public CANifierAccess() {}

  public boolean algaeManiSensor()
  {
    SD.put(Key.SENSOR_ALGAE, io_CANifier.getGeneralInput(IDConstants.algaeManipulatorDIO));
    return io_CANifier.getGeneralInput(IDConstants.algaeManipulatorDIO);
  }

  public boolean coralManiStbdSensor()
  {
    SD.put(Key.SENSOR_CORAL1, io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOStbd));
    return io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOStbd);
  }

  public boolean coralManiPortSensor()
  {
    SD.put(Key.SENSOR_CORAL2, io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOPort));
    return io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOPort);
  }
}

// io_CANifier.getGeneralInput(GeneralPin.SPI_CLK_PWM0P);  PWM1, PWM2, PWM0
