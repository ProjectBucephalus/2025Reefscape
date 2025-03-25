// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.constants.IDConstants;
import frc.robot.util.SD;

import com.ctre.phoenix.CANifier;

public class CANifierAccess
{
  CANifier canifierIO = new CANifier(IDConstants.canifierID);

  public CANifierAccess() {}

  public boolean coralStbdSensor()
  {
    SD.SENSOR_CORAL1.put(canifierIO.getGeneralInput(IDConstants.coralSensorDIOStbd));
    return canifierIO.getGeneralInput(IDConstants.coralSensorDIOStbd);
  }

  public boolean coralPortSensor()
  {
    SD.SENSOR_CORAL2.put(canifierIO.getGeneralInput(IDConstants.coralSensorDIOPort));
    return canifierIO.getGeneralInput(IDConstants.coralSensorDIOPort);
  }
}
