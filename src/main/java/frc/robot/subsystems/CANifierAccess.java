// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.constants.IDConstants;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CANifierAccess
{
  CANifier io_CANifier = new CANifier(IDConstants.canifierID);

  public CANifierAccess() {}

  public boolean algaeManiSensor()
  {
    SmartDashboard.putBoolean("A Beam", io_CANifier.getGeneralInput(IDConstants.algaeManipulatorDIO));
    return io_CANifier.getGeneralInput(IDConstants.algaeManipulatorDIO);
  }

  public boolean coralManiStbdSensor()
  {
    SmartDashboard.putBoolean("C Beam 1", io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOStbd));
    return io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOStbd);
  }

  public boolean coralManiPortSensor()
  {
    SmartDashboard.putBoolean("C Beam 2", io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOPort));
    return io_CANifier.getGeneralInput(IDConstants.coralManipulatorDIOPort);
  }
}
