// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Simplified interface for most SmartDashboard interactions */
public class SD 
{
  public static final BooleanKey IO_LL = new BooleanKey("Use Limelight", true);
  public static final DoubleKey IO_LL_EXPOSURE = new DoubleKey("Exposure Setting", 0);

  public static final BooleanKey CALIBRATE_BOT_ROTATION = new BooleanKey("Rotation Known", false);
  public static final BooleanKey CALIBRATE_DIFF = new BooleanKey("Overide: Calibrate Arm", false);
  public static final BooleanKey CALIBRATE_DIFF_TARGET = new BooleanKey("Overide: Arm At Target", false);

  public static final StringKey STATE_HEADING = new StringKey("Heading State", "");

  public static final BooleanKey IO_PROCESS_AUTO = new BooleanKey("Process Auto", false);
  public static final BooleanKey IO_GEOFENCE = new BooleanKey("IgnoreFence", false);
  public static final StringKey IO_AUTO = new StringKey("Auto Input", Constants.Auto.defaultAuto);
  public static final DoubleKey IO_RUMBLE_D = new DoubleKey("Driver Rumble", Constants.RumblerConstants.driverDefault);
  public static final DoubleKey IO_RUMBLE_C = new DoubleKey("Copilot Rumble", Constants.RumblerConstants.copilotDefault);

  public static final BooleanKey STATE_PP_WARMUP = new BooleanKey("Warmup Finished", false);
  public static final BooleanKey STATE_RED = new BooleanKey("redAlliance", false);
  public static final StringKey STATE_DRIVE = new StringKey("Drive State", "Disabled");
  public static final BooleanKey STATE_HEADING_SNAP = new BooleanKey("Heading Snap Updating", true);

  public static final DoubleKey BOT_SPEED = new DoubleKey("Robot Speed", 0);

  public static final BooleanKey SENSOR_ALGAE = new BooleanKey("A Beam", false);
  public static final BooleanKey SENSOR_CORAL1 = new BooleanKey("C Beam 1", false);
  public static final BooleanKey SENSOR_CORAL2 = new BooleanKey("C Beam 2", false);
  public static final DoubleKey SENSOR_GYRO = new DoubleKey("Gyro yaw", 0);
  public static final DoubleKey SENSOR_DIFF_ELEVATION = new DoubleKey("Potentiometer Reading", 0);
  public static final DoubleKey SENSOR_DIFF_ANGLE = new DoubleKey("Encoder Reading", 0);
  public static final DoubleKey SENSOR_DIFF_POT = new DoubleKey("Elevator Potentiometer", 0);

  public static final DoubleKey CLIMBER_POS = new DoubleKey("Climber Position", 0);
  public static final DoubleKey CLIMBER_TARGET = new DoubleKey("Climber Target", 0);

  public static final DoubleKey DIFF_ELEVATION = new DoubleKey("Elevator Height", 0);
  public static final DoubleKey DIFF_ELEVATION_TARGET = new DoubleKey("Elevator Target", 0);
  public static final DoubleKey DIFF_ANGLE = new DoubleKey("Arm Rotation", 0);
  public static final DoubleKey DIFF_ANGLE_TARGET = new DoubleKey("Arm Target", 0);
  public static final DoubleKey DIFF_UA_ER = new DoubleKey("UA Error", 0);
  public static final DoubleKey DIFF_DA_ER = new DoubleKey("DA Error", 0);  
  public static final DoubleKey DIFF_HEIGHT = new DoubleKey("Height over deck", 0);  
  public static final DoubleKey DIFF_ANGLE_ER = new DoubleKey("Offset", 0);

  public static final StringKey RUMBLE_D_R = new StringKey("DriverRight Rumble Queue", "");
  public static final StringKey RUMBLE_D_L = new StringKey("DriverLeft Rumble Queue", "");
  public static final StringKey RUMBLE_C_R = new StringKey("CopilotRight Rumble Queue", "");
  public static final StringKey RUMBLE_C_L = new StringKey("CopilotLeft Rumble Queue", "");

  public static final BooleanKey DIFF_ESTOP = new BooleanKey("Diffector E-Stop", false);
  public static final BooleanKey OVERRIDE = new BooleanKey("OVERIDE MODE", false);

  public record BooleanKey (String label, boolean defaultValue) implements Runnable, Supplier<Boolean>, Consumer<Boolean>
  {
    @Override
    public Boolean get() {return SmartDashboard.getBoolean(label, defaultValue);}

    @Override
    public void run() {SmartDashboard.putBoolean(label, defaultValue);}

    @Override
    public void accept(Boolean value) {SmartDashboard.putBoolean(label, value);}

    public void init() {run();}

    public void put(boolean value) {accept(value);}
  }

  public record DoubleKey (String label, double defaultValue) implements Runnable, Supplier<Double>, Consumer<Double>
  {
    @Override
    public Double get() {return SmartDashboard.getNumber(label, defaultValue);}

    @Override
    public void run() {SmartDashboard.putNumber(label, defaultValue);}

    @Override
    public void accept(Double value) {SmartDashboard.putNumber(label, value);}

    public void init() {run();}

    public void put(double value) {accept(value);}
  }

  public record StringKey (String label, String defaultValue) implements Runnable, Supplier<String>, Consumer<String>
  {
    @Override
    public String get() {return SmartDashboard.getString(label, defaultValue);}

    @Override
    public void run() {SmartDashboard.putString(label, defaultValue);}

    @Override
    public void accept(String value) {SmartDashboard.putString(label, value);}

    public void init() {run();}

    public void put(String value) {accept(value);}
  }
}
