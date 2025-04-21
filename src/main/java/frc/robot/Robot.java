// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.util.FieldUtils;
import frc.robot.util.SD;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Logged
public class Robot extends TimedRobot 
{
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private Pose2d robotPose;

  private Command warmupCommand;
  
  private boolean allianceKnown = false;

  public Robot()
  {
    robotContainer = new RobotContainer();
    warmupCommand = PathfindingCommand.warmupCommand();
    warmupCommand.schedule();

    RobotContainer.s_Swerve.getPigeon2().setYaw(0);

    RobotContainer.io_LimelightPort.setThrottle(0);
    RobotContainer.io_LimelightStbd.setThrottle(0);
    RobotContainer.io_LimelightPort.setIMUMode(1);
    RobotContainer.io_LimelightStbd.setIMUMode(1);
    SignalLogger.enableAutoLogging(false);

    DataLogManager.start("/home/lvuser/logs");
    DriverStation.startDataLog(DataLogManager.getLog());
    Epilogue.bind(this);
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
    
    CommandScheduler.getInstance().run();
    
    SD.STATE_HEADING.put(RobotContainer.headingState.toString());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() 
  {
    RobotContainer.io_LimelightPort.setIMUMode(1);
    RobotContainer.io_LimelightStbd.setIMUMode(1);

    SD.OVERRIDE.init();
    SD.IO_PROCESS_AUTO.init();

    RobotContainer.io_copilotLeft.clearRequests();
    RobotContainer.io_copilotRight.clearRequests();
    RobotContainer.io_driverLeft.clearRequests();
    RobotContainer.io_driverRight.clearRequests();
  }

  @Override
  public void disabledPeriodic()
  {
    SD.STATE_PP_WARMUP.put(!warmupCommand.isScheduled());

    if (SD.IO_PROCESS_AUTO.button())
    {
      autonomousCommand = robotContainer.getAutoCommand();
    }
    
    if (!allianceKnown) 
    {
      if (DriverStation.getAlliance().isPresent()) 
      {
        allianceKnown = true;
        allianceInit();
      }  
    }
  }

  @Override
  public void autonomousInit() 
  {     
    RobotContainer.io_LimelightPort.setIMUMode(1);
    RobotContainer.io_LimelightStbd.setIMUMode(1);

    RobotContainer.s_Swerve.resetPose(new Pose2d(RobotContainer.swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(RobotContainer.s_Swerve.getPigeon2().getYaw().getValueAsDouble()))));
    
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
    RobotContainer.io_LimelightPort.setIMUMode(1);
    RobotContainer.io_LimelightStbd.setIMUMode(1);

    if (autonomousCommand != null) 
      {autonomousCommand.cancel();}

    RobotContainer.s_Swerve.resetPose(new Pose2d(RobotContainer.swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(RobotContainer.s_Swerve.getPigeon2().getYaw().getValueAsDouble()))));

    RobotContainer.s_Coral.setStatus(CoralManipulator.Status.DEFAULT);
    RobotContainer.s_Algae.setStatus(AlgaeManipulator.Status.EMPTY);

    RobotContainer.io_driverLeft.timedRequestCommand("Teleop Start", 1.5).schedule();
    RobotContainer.io_driverRight.timedRequestCommand("Teleop Start", 1.5).schedule();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() 
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.io_LimelightPort.setThrottle(0);
    RobotContainer.io_LimelightStbd.setThrottle(0);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public static void allianceInit() 
  {
    ArrayList<Pair<Translation2d, Translation2d>> bargeObstacle = new ArrayList<Pair<Translation2d, Translation2d>>();
    bargeObstacle.add(FieldUtils.isRedAlliance() ? FieldUtils.GeoFencing.redAllianceBargeDynamic : FieldUtils.GeoFencing.blueAllianceBargeDynamic);

    Pathfinding.setDynamicObstacles(bargeObstacle, RobotContainer.swerveState.Pose.getTranslation());

    if (DriverStation.getAlliance().get() == Alliance.Blue && !Limelight.rotationKnown) 
      {RobotContainer.s_Swerve.getPigeon2().setYaw(180);}
  }
}
