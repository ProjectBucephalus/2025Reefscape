// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.constants.IDConstants;
import frc.robot.util.SD;

import com.ctre.phoenix.CANifier;

public class CANifierAccess
{
  CANifier io_CANifier = new CANifier(IDConstants.canifierID);

  public CANifierAccess() {}

  public boolean coralManiStbdSensor()
  {
    SD.SENSOR_CORAL1.put(io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOStbd));
    return io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOStbd);
  }

  public boolean coralManiPortSensor()
  {
    SD.SENSOR_CORAL2.put(io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOPort));
    return io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOPort);
  }
}
