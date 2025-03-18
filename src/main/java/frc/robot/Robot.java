// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    RobotContainer.limelightPort.setIMUMode(1);
    RobotContainer.limelightStbd.setIMUMode(1);
    SD.IO_LL_EXPOSURE.init();
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
        RobotContainer.swerve.resetPose(new Pose2d((FieldUtils.fieldLength/2) - 1.5, FieldUtils.fieldWidth/2, robotPose.getRotation()));
      else
        RobotContainer.swerve.resetPose(new Pose2d((FieldUtils.fieldLength/2) + 1.5, FieldUtils.fieldWidth/2, robotPose.getRotation()));
    }

    RobotContainer.swerveState = RobotContainer.swerve.getState();

    RobotContainer.swerve.resetPose(new Pose2d(RobotContainer.swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(RobotContainer.swerve.getPigeon2().getYaw().getValueAsDouble()))));
    
    CommandScheduler.getInstance().run();
    
    SD.STATE_HEADING.put(RobotContainer.headingState.toString());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() 
  {
    RobotContainer.limelightPort.setIMUMode(1);
    RobotContainer.limelightStbd.setIMUMode(1);
    if (Limelight.rotationKnown)
    {
      RobotContainer.limelightPort.setThrottle(150);
      RobotContainer.limelightStbd.setThrottle(150);
    }
    SD.OVERRIDE.init();
    SD.IO_PROCESS_AUTO.init();
    SD.CALIBRATE_BOT_ROTATION.init();
    Limelight.rotationKnown = false;
  }

  @Override
  public void disabledPeriodic()
  {
    SD.STATE_PP_WARMUP.put(!warmupCommand.isScheduled());

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
        if (DriverStation.getAlliance().get() == Alliance.Blue && !Limelight.rotationKnown) 
          {RobotContainer.swerve.getPigeon2().setYaw(180);}
      }  
    }
  }

  @Override
  public void autonomousInit() 
  {  
    RobotContainer.limelightPort.setIMUMode(2);
    RobotContainer.limelightStbd.setIMUMode(2);    
    RobotContainer.limelightPort.setThrottle(0);
    RobotContainer.limelightStbd.setThrottle(0);
    
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
    RobotContainer.limelightPort.setIMUMode(2);
    RobotContainer.limelightStbd.setIMUMode(2);    
    RobotContainer.limelightPort.setThrottle(0);
    RobotContainer.limelightStbd.setThrottle(0);

    if (autonomousCommand != null) 
      {autonomousCommand.cancel();}

    RobotContainer.coralManip.setStatus(CoralManipulator.Status.DEFAULT);
    RobotContainer.algaeManip.setStatus(AlgaeManipulator.Status.EMPTY);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() 
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.limelightPort.setThrottle(0);
    RobotContainer.limelightStbd.setThrottle(0);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
