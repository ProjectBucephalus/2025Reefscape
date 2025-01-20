// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControllerLayering extends SubsystemBase {
  /** Creates a new ControllerLayering. */
  public ControllerLayering() {}

  public void f1(boolean out)
  {
    SmartDashboard.putBoolean("f1", out);
  }

  public void f2(boolean out)
  {
    SmartDashboard.putBoolean("f2", out);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
