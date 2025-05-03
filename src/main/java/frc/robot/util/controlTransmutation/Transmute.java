// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants.Control;

/** 
 * Class for handling and modifying all joystick input values. 
 * Including deadbanding, braking, response-curves, geofencing, and more.
 * 
 * @author 5985
 */
public class Transmute 
{
  public static Translation2d linearDeadband(Translation2d input, double deadband)
  {
    if (input.getNorm() <= deadband) 
      {return Translation2d.kZero;}
    else
      {return input;}
  }

  public static Translation2d driverDeadband(Translation2d input)
    {return linearDeadband(input, Control.stickDeadband);}
  
  /**
   * Snaps the input to be purely cardinal
   * @param input Stick input with 2 axis [-1..1]
   * @param separation Determines the size and behaviour of corners: <1 deadzone, >1 smooth control 
   * @return 
   */
  public static Translation2d cardinalLock(Translation2d input, double separation)
  {
    return new Translation2d
    (
      Math.abs(input.getX()) < separation * Math.abs(input.getY()) ? 0 : input.getX(),
      Math.abs(input.getY()) < separation * Math.abs(input.getX()) ? 0 : input.getY()  
    );
  }

  public static Translation2d cardinalLock(Translation2d input)
    {return cardinalLock(input, 1);}
}
