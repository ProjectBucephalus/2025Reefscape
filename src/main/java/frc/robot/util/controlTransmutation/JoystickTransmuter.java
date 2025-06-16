// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.controlTransmutation;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class JoystickTransmuter extends InputTransmuter
{
  private InputCurve inputCurve;
  private Deadband deadband;
  private Brake brake;
  private ObjectList objectList;

  private DoubleSupplier inputX;
  private DoubleSupplier inputY;

  public final Supplier<Translation2d> stickOutputSup;

  public JoystickTransmuter(DoubleSupplier inputX, DoubleSupplier inputY)
  {
    this.inputX = inputX;
    this.inputY = inputY;

    inputCurve = new InputCurve();
    deadband = new Deadband();
    brake = new Brake();
    objectList = new ObjectList();

    stickOutputSup = () -> stickOutput();
  }

  public Translation2d stickOutput()
  {
    return process(new Translation2d(inputX.getAsDouble(), inputY.getAsDouble()));
  }

  public Translation2d process(Translation2d controlInput)
  {
    return 
    objectList.process
    (
      brake.process
      (
        inputCurve.process
        (
          deadband.process
          (
            controlInput
          )
        )
      )
    );
  }

  public JoystickTransmuter withInputCurve(InputCurve inputCurve)
  {
    this.inputCurve = inputCurve;
    return this;
  }

  public JoystickTransmuter withDeadband(Deadband deadband)
  {
    this.deadband = deadband;
    return this;
  }

  public JoystickTransmuter withBrake(Brake brake)
  {
    this.brake = brake;
    return this;
  }

  public JoystickTransmuter withObjectList(ObjectList objectList)
  {
    this.objectList = objectList;
    return this;
  }
}
