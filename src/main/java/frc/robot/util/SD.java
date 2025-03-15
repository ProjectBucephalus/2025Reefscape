// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Simplified interface for most SmartDashboard interactions */
public class SD 
{
  public static enum DataType
  {
    STRING,
    NUMBER,
    BOOLEAN
  }

  public static enum Key
  {
    IO_LL("Use Limelight", true),
    IO_LL_EXPOSURE("Exposure Setting", 0),

    CALIBRATE_BOT_ROTATION("Rotation Known", false),
    CALIBRATE_DIFF("Overide: Calibrate Arm", false),
    CALIBRATE_DIFF_TARGET("Overide: Arm At Target", false),

    STATE_HEADING("Heading State", ""),

    IO_PROCESS_AUTO("Process Auto", false),
    IO_GEOFENCE("IgnoreFence", false),
    IO_AUTO("Auto Input", Constants.Auto.defaultAuto),
    IO_RUMBLE_D("Driver Rumble", Constants.RumblerConstants.driverDefault),
    IO_RUMBLE_C("Copilot Rumble", Constants.RumblerConstants.copilotDefault),

    STATE_PP_WARMUP("Warmup Finished", false),
    STATE_RED("redAlliance", false),
    STATE_DRIVE("Drive State", "Disabled"),
    STATE_HEADING_SNAP("Heading Snap Updating", true),

    BOT_SPEED("Robot Speed", 0),

    SENSOR_ALGAE("A Beam", false),
    SENSOR_CORAL1("C Beam 1", false),
    SENSOR_CORAL2("C Beam 2", false),
    SENSOR_GYRO("Gyro yaw", 0),
    SENSOR_DIFF_ELEVATION("Potentiometer Reading", 0),
    SENSOR_DIFF_ANGLE("Encoder Reading", 0),

    CLIMBER_POS("Climber Position", 0),
    CLIMBER_TARGET("Climber Target", 0),

    DIFF_ELEVATION("Elevator Height", 0),
    DIFF_ELEVATION_TARGET("Elevator Target", 0),
    DIFF_ANGLE("Arm Rotation", 0),
    DIFF_ANGLE_TARGET("Arm Target", 0),
    DIFF_UA_ER("UA Error", 0),  
    DIFF_DA_ER("DA Error", 0),  
    DIFF_HEIGHT("Height over deck", 0),  
    DIFF_ANGLE_ER("Offset", 0),

    RUMBLE_D_R("DriverRight Rumble Queue", ""),
    RUMBLE_D_L("DriverLeft Rumble Queue", ""),
    RUMBLE_C_R("CopilotRight Rumble Queue", ""),
    RUMBLE_C_L("CopilotLeft Rumble Queue", ""),

    DIFF_ESTOP("Diffector E-Stop", false),
    OVERIDE("OVERIDE MODE", false);

    public final DataType dataType;
    public final String label;
    private final String defaultString;
    private final double defualtNumber;
    private final boolean defaultBoolean;

    private Key(String label, String defaultValue)
    {
      this.dataType = DataType.STRING;
      this.label = label;
      this.defaultString = defaultValue;
      
      this.defualtNumber = 0;
      this.defaultBoolean = false;
    }

    private Key(String label, double defaultValue)
    {
      this.dataType = DataType.NUMBER;
      this.label = label;
      this.defualtNumber = defaultValue;
      
      this.defaultString = "";
      this.defaultBoolean = false;
    }

    private Key(String label, boolean defaultValue)
    {
      this.dataType = DataType.BOOLEAN;
      this.label = label;
      this.defaultBoolean = defaultValue;
      
      this.defaultString = "";
      this.defualtNumber = 0;
    }
  }

  /**
   * Initialises the SmartDashboard field with the default value
   * @param key Reference key tied to a SmartDashboard label
   */
  public static void init(Key key)
  {
    switch (key.dataType) 
    {
      case STRING:
        SmartDashboard.putString(key.label, key.defaultString);
        break;

      case NUMBER:
        SmartDashboard.putNumber(key.label, key.defualtNumber);
        break;

      case BOOLEAN:
        SmartDashboard.putBoolean(key.label, key.defaultBoolean);
        break;
    
      default:
        break;
    }
  }

  /**
   * Send data to SmartDashboard
   * @param key Reference key tied to a SmartDashboard label
   * @param value String, Number, or Boolean value to send, as appropriate
   */
  public static void put(Key key, String value)
  {
    if (key.dataType == DataType.STRING)
      SmartDashboard.putString(key.label, value);
  }

  /**
   * Send data to SmartDashboard
   * @param key Reference key tied to a SmartDashboard label
   * @param value String, Number, or Boolean value to send, as appropriate
   */
  public static void put(Key key, double value)
  {
    if (key.dataType == DataType.NUMBER)
      SmartDashboard.putNumber(key.label, value);
  }

  /**
   * Send data to SmartDashboard
   * @param key Reference key tied to a SmartDashboard label
   * @param value String, Number, or Boolean value to send, as appropriate
   */
  public static void put(Key key, boolean value)
  {
    if (key.dataType == DataType.BOOLEAN)
      SmartDashboard.putBoolean(key.label, value);
  }

  /**
   * Pull String from SmartDashboard
   * @param key Reference key tied to a SmartDashboard label
   * @return String found at associated index, otherwise returns empty string
   */
  public static String getString(Key key)
  {
    if (key.dataType == DataType.STRING)
      return SmartDashboard.getString(key.label, key.defaultString);
    else return "";
  }

  /**
   * Pull Number from SmartDashboard
   * @param key Reference key tied to a SmartDashboard label
   * @return Number (double) found at associated index, otherwise returns 0
   */
  public static double getNumber(Key key)
  {
    if (key.dataType == DataType.NUMBER)
      return SmartDashboard.getNumber(key.label, key.defualtNumber);
    else return 0;
  }

  /**
   * Pull Boolean from SmartDashboard
   * @param key Reference key tied to a SmartDashboard label
   * @return Boolean found at associated index, otherwise returns false
   */
  public static boolean getBoolean(Key key)
  {
    if (key.dataType == DataType.BOOLEAN)
      return SmartDashboard.getBoolean(key.label, key.defaultBoolean);
    else return false;
  }
}
