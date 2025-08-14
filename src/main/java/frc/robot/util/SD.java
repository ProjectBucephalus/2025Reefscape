// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Set;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.MechanismConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Simplified interface for most SmartDashboard interactions */
public abstract class SD 
{
  public static final BooleanKey IO_LL = new BooleanKey("Use Limelight", true);
  public static final DoubleKey  IO_LL_EXPOSURE = new DoubleKey("Exposure Setting", 0);
  public static final BooleanKey IO_LL_EXPOSURE_UP = new BooleanKey("Increase Exposure", false);
  public static final BooleanKey IO_LL_EXPOSURE_DOWN = new BooleanKey("Decrease Exposure", false);
  public static final DoubleKey  IO_LED_BRIGHTNESS = new DoubleKey("LED Brightness", 1);
  public static final DoubleKey  IO_CLIMB_WARNING = new DoubleKey("Climb Warning Time", 27);

  public static final BooleanKey ROTATION_KNOWN = new BooleanKey("Rotation Known", false);
  public static final BooleanKey CALIBRATE_DIFF = new BooleanKey("Overide: Calibrate Arm", false);
  public static final BooleanKey CALIBRATE_DIFF_TARGET = new BooleanKey("Overide: Arm At Target", false);

  public static final StringKey  STATE_HEADING = new StringKey("Heading State", "");
  
  public static final StringKey  STATE_LED_BAR = new StringKey("LEDs Bar", "");
  public static final StringKey  STATE_LED_HAL = new StringKey("LEDs Halo", "");
  public static final StringKey  STATE_LED_ALL = new StringKey("LEDs All", "");

  public static final DoubleKey  IO_ALGAE_HOLD = new DoubleKey("Algae Holding Value", MechanismConstants.AlgaeConfigs.algaeHoldingCurrent);
  public static final BooleanKey IO_PROCESS_AUTO = new BooleanKey("Process Auto", false);
  public static final BooleanKey IO_GEOFENCE = new BooleanKey("Use Fence", true);
  public static final BooleanKey IO_OUTER_GEOFENCE = new BooleanKey("Wall Fence", true);
  public static final DoubleKey  IO_GEOFENCE_IMPACT = new DoubleKey("Fence Impact", 1);
  public static final StringKey  IO_AUTO = new StringKey("Auto Input", Constants.Auto.defaultAuto);
  public static final DoubleKey  IO_RUMBLE_D = new DoubleKey("Driver Rumble", Constants.RumblerConstants.driverDefault);
  public static final DoubleKey  IO_RUMBLE_C = new DoubleKey("Copilot Rumble", Constants.RumblerConstants.copilotDefault);
  public static final BooleanKey IO_BARGE_PROTECTION = new BooleanKey("Use Barge Protection", true);

  public static final BooleanKey STATE_PP_WARMUP = new BooleanKey("Warmup Finished", false);
  public static final StringKey  STATE_ALGAE = new StringKey("Algae Manipulator State", "Empty");
  public static final StringKey  STATE_DRIVE = new StringKey("Drive State", "Disabled");
  public static final BooleanKey STATE_HEADING_SNAP = new BooleanKey("Heading Snap Updating", true);

  public static final DoubleKey SENSOR_ALGAE_TMEP = new DoubleKey("Algae Temperature", 0);
  public static final DoubleKey SENSOR_ALGAE_CURRENT = new DoubleKey("Algae Current", 0);
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

  public static final DoubleKey  IO_DIFF_ELEVATION = new DoubleKey ("Manual Elevation Target", 1);
  public static final DoubleKey  IO_DIFF_ANGLE     = new DoubleKey ("Manual Angle Target", 0);
  public static final BooleanKey IO_DIFF_GOTO      = new BooleanKey("Arm To Manual Target", false);

  public static final StringKey RUMBLE_D_R = new StringKey("DriverRight Rumble Queue", "");
  public static final StringKey RUMBLE_D_L = new StringKey("DriverLeft Rumble Queue", "");
  public static final StringKey RUMBLE_C_R = new StringKey("CopilotRight Rumble Queue", "");
  public static final StringKey RUMBLE_C_L = new StringKey("CopilotLeft Rumble Queue", "");

  public static final BooleanKey DIFF_ESTOP = new BooleanKey("Diffector E-Stop", false);
  public static final BooleanKey OVERRIDE = new BooleanKey("OVERIDE MODE", false);
  public static final BooleanKey CLIMB_OVERRIDE = new BooleanKey("Override Climber", false);

  public static final BooleanKey IO_POSE_PATHFIND = new BooleanKey("Pathfind to selected Pose", false);
  public static final DoubleKey IO_POSE_X = new DoubleKey("Pose X", 0.0);
  public static final DoubleKey IO_POSE_Y = new DoubleKey("Pose Y", 0.0);
  public static final DoubleKey IO_POSE_R = new DoubleKey("Pose Rotation", 0.0);

  public static final BooleanKey STATE_DEMO = new BooleanKey("Demo Mode", false);

  static
  {
    for 
    (
      Initable key : 
      Set.of
      (
        IO_LL_EXPOSURE_UP,
        IO_LL_EXPOSURE_DOWN,
        ROTATION_KNOWN,
        OVERRIDE,
        IO_PROCESS_AUTO,
        IO_GEOFENCE,
        CLIMB_OVERRIDE,
        DIFF_ESTOP,
        IO_BARGE_PROTECTION,
        CALIBRATE_DIFF,
        CALIBRATE_DIFF_TARGET,
        IO_LL,
        IO_POSE_PATHFIND,
        IO_DIFF_GOTO,
        IO_LL_EXPOSURE,
        IO_AUTO,
        IO_ALGAE_HOLD,
        IO_LED_BRIGHTNESS,
        IO_GEOFENCE_IMPACT,
        IO_POSE_X,
        IO_POSE_Y,
        IO_POSE_R,
        IO_DIFF_ANGLE,
        IO_DIFF_ELEVATION,
        IO_CLIMB_WARNING,
        STATE_LED_BAR,
        STATE_LED_HAL,
        STATE_LED_ALL,
        IO_OUTER_GEOFENCE,
        STATE_DEMO
      )
    )
    {
      key.init();
    }
  }

  public static void initSwerveDisplay(CommandSwerveDrivetrain s_Swerve)
  {
    SmartDashboard.putData
    (
      "Swerve Drive", 
      new Sendable() 
      {
        @Override
        public void initSendable(SendableBuilder builder) 
        {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty("Front Left Angle", () -> s_Swerve.getModule(0).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Front Left Velocity", () -> s_Swerve.getModule(0).getCurrentState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Front Right Angle", () -> s_Swerve.getModule(1).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Front Right Velocity", () -> s_Swerve.getModule(1).getCurrentState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Left Angle", () -> s_Swerve.getModule(2).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Back Left Velocity", () -> s_Swerve.getModule(2).getCurrentState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Right Angle", () -> s_Swerve.getModule(3).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Back Right Velocity", () -> s_Swerve.getModule(3).getCurrentState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Robot Angle", () -> RobotContainer.swerveState.Pose.getRotation().getRadians(), null);
        }
      }
    );
  }

  public interface Initable
  {
    public void init();
  }

  public record BooleanKey (String label, boolean defaultValue) implements Initable
  {
    public boolean get() {return SmartDashboard.getBoolean(label, defaultValue);}

    public boolean button() 
    {
      if (SmartDashboard.getBoolean(label, defaultValue)) 
      {
        put(false); 
        return true;
      } 
      else 
        {return false;}
    }

    public void init() {SmartDashboard.putBoolean(label, defaultValue);}

    public void put(boolean value) {SmartDashboard.putBoolean(label, value);}
  }

  public record DoubleKey (String label, double defaultValue) implements Initable
  {
    public Double get() {return SmartDashboard.getNumber(label, defaultValue);}

    public void init() {SmartDashboard.putNumber(label, defaultValue);}

    public void put(double value) {SmartDashboard.putNumber(label, value);}
  }

  public record StringKey (String label, String defaultValue) implements Initable
  {
    public String get() {return SmartDashboard.getString(label, defaultValue);}

    public void init() {SmartDashboard.putString(label, defaultValue);}

    public void put(String value) {SmartDashboard.putString(label, value);}
  }
}
