// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.CTREConfigs;
import frc.robot.subsystems.AlgaeManipulator.AlgaeStatus;
import frc.robot.subsystems.CoralManipulator.CoralStatus;
import frc.robot.util.FieldUtils;
import frc.robot.util.SD;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private Field2d autoPosition = new Field2d();

  private Pose2d robotPose;

  private Command c_WarmupCommand;

  private boolean allianceKnown = false;
  private boolean rotationKnown = false;
  private ArrayList<Double> portRotationData = new ArrayList<Double>();
  private ArrayList<Double> stbdRotationData = new ArrayList<Double>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    robotContainer = new RobotContainer();
    c_WarmupCommand = PathfindingCommand.warmupCommand();

    c_WarmupCommand.schedule();

    RobotContainer.s_LimelightPort.setIMUMode(1);
    RobotContainer.s_LimelightStbd.setIMUMode(1);

    SmartDashboard.putData("Field", autoPosition);
    SD.IO_LL_EXPOSURE.init();
    SD.IO_LL_EXPOSURE_UP.init();
    SD.IO_LL_EXPOSURE_DOWN.init();
    SD.CALIBRATE_BOT_ROTATION.init();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    robotPose = RobotContainer.swerveState.Pose;

    if (robotPose.getX() <= 0.25 && robotPose.getY() <= 0.25) 
    {
      if (allianceKnown && DriverStation.getAlliance().get() == Alliance.Blue)
        RobotContainer.s_Swerve.resetPose(new Pose2d((FieldUtils.fieldLength/2) - 1.5, FieldUtils.fieldWidth/2, robotPose.getRotation()));
      else
        RobotContainer.s_Swerve.resetPose(new Pose2d((FieldUtils.fieldLength/2) + 1.5, FieldUtils.fieldWidth/2, robotPose.getRotation()));
    }

    RobotContainer.swerveState = RobotContainer.s_Swerve.getState();

    rotationKnown = SD.CALIBRATE_BOT_ROTATION.get();

    if (!rotationKnown)
    {
      if (!RobotContainer.s_LimelightPort.getLimelightRotation().equals(Rotation2d.kZero))
      {
        portRotationData.add(0, RobotContainer.s_LimelightPort.getLimelightRotation().getDegrees());

        if (portRotationData.size() > 5)
          {portRotationData.remove(5);}

        if (portRotationData.size() == 5)
        {
          double lowest = portRotationData.get(0).doubleValue();
          double highest = portRotationData.get(0).doubleValue();
          
          for(int i = 1; i < 5; i++)
          {
            lowest = Math.min(lowest, portRotationData.get(i).doubleValue());
            highest = Math.max(highest, portRotationData.get(i).doubleValue());
          }
          
          if (highest - lowest < 1)
          {
            RobotContainer.s_Swerve.getPigeon2().setYaw((highest + lowest) / 2);
            SD.CALIBRATE_BOT_ROTATION.put(true);
            portRotationData.clear();
            stbdRotationData.clear();
            RobotContainer.s_LimelightPort.setThrottle(150);
            RobotContainer.s_LimelightStbd.setThrottle(150);
          }

        }
      }

      if (!RobotContainer.s_LimelightStbd.getLimelightRotation().equals(Rotation2d.kZero))
      {
        stbdRotationData.add(0, RobotContainer.s_LimelightStbd.getLimelightRotation().getDegrees());

        if (stbdRotationData.size() > 5)
          {stbdRotationData.remove(5);}

        else if (stbdRotationData.size() == 5)
        {
          double lowest = stbdRotationData.get(0).doubleValue();
          double highest = stbdRotationData.get(0).doubleValue();
          
          for(int i = 1; i < 5; i++)
          {
            lowest = Math.min(lowest, stbdRotationData.get(i).doubleValue());
            highest = Math.max(highest, stbdRotationData.get(i).doubleValue());
          }

          if (highest - lowest < 1)
          {
            RobotContainer.s_Swerve.getPigeon2().setYaw((highest + lowest) / 2);
            SD.CALIBRATE_BOT_ROTATION.put(true);
            portRotationData.clear();
            stbdRotationData.clear();
            RobotContainer.s_LimelightPort.setThrottle(150);
            RobotContainer.s_LimelightStbd.setThrottle(150);
          }
        }
      }
    }

    RobotContainer.s_Swerve.resetPose(new Pose2d(RobotContainer.swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(RobotContainer.s_Swerve.getPigeon2().getYaw().getValueAsDouble()))));

    CommandScheduler.getInstance().run();

    SD.STATE_HEADING.put(RobotContainer.headingState.toString());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() 
  {
    RobotContainer.s_LimelightPort.setIMUMode(1);
    RobotContainer.s_LimelightStbd.setIMUMode(1);
    if (rotationKnown)
    {
      RobotContainer.s_LimelightPort.setThrottle(150);
      RobotContainer.s_LimelightStbd.setThrottle(150);
    }
    SD.OVERRIDE.init();
    SD.IO_PROCESS_AUTO.init();
    SD.CALIBRATE_BOT_ROTATION.init();
    rotationKnown = false;
  }

  @Override
  public void disabledPeriodic()
  {
    SD.STATE_PP_WARMUP.put(!c_WarmupCommand.isScheduled());

    if (SD.IO_PROCESS_AUTO.get())
    {
      autonomousCommand = robotContainer.getAutoCommand();
      SD.IO_PROCESS_AUTO.put(false);
    }

    if (!allianceKnown) 
    {
      if (DriverStation.getAlliance().isPresent()) 
      {
        allianceKnown = true;
        if (DriverStation.getAlliance().get() == Alliance.Blue && !rotationKnown) 
          {RobotContainer.s_Swerve.getPigeon2().setYaw(180);}
      }  
    }
  }

  @Override
  public void autonomousInit() 
  {  
    RobotContainer.s_LimelightPort.setIMUMode(2);
    RobotContainer.s_LimelightStbd.setIMUMode(2);    
    RobotContainer.s_LimelightPort.setThrottle(0);
    RobotContainer.s_LimelightStbd.setThrottle(0);
    
    if (autonomousCommand == null) 
      {autonomousCommand = robotContainer.getAutoCommand();}

    if (autonomousCommand != null) 
      {autonomousCommand.schedule();}
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    RobotContainer.s_LimelightPort.setIMUMode(2);
    RobotContainer.s_LimelightStbd.setIMUMode(2);    
    RobotContainer.s_LimelightPort.setThrottle(0);
    RobotContainer.s_LimelightStbd.setThrottle(0);

    if (autonomousCommand != null) 
      {autonomousCommand.cancel();}

    RobotContainer.s_Coral.setStatus(CoralStatus.DEFAULT);
    RobotContainer.s_Algae.setStatus(AlgaeStatus.EMPTY);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() 
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.s_LimelightPort.setThrottle(0);
    RobotContainer.s_LimelightStbd.setThrottle(0);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
