// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.CTREConfigs;
import frc.robot.util.FieldUtils;

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

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    robotContainer = new RobotContainer();
    PathfindingCommand.warmupCommand().schedule();

    RobotContainer.s_LimelightPort.setIMUMode(1);
    RobotContainer.s_LimelightStbd.setIMUMode(1);

    SmartDashboard.putData("Field", autoPosition);
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
    CommandScheduler.getInstance().run();

    if (RobotContainer.s_Swerve.getState().Pose.getX() < 0.75 && RobotContainer.s_Swerve.getState().Pose.getY() < 0.75) 
      {RobotContainer.s_Swerve.resetPose(new Pose2d(1.5, 1, RobotContainer.s_Swerve.getState().Pose.getRotation()));}

    SmartDashboard.putBoolean("Unlock Heading Trigger", RobotContainer.unlockHeadingTrigger.getAsBoolean());
    SmartDashboard.putString("Heading State", RobotContainer.headingState.toString());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() 
  {
    RobotContainer.s_LimelightPort.setIMUMode(1);
    RobotContainer.s_LimelightStbd.setIMUMode(1);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() 
  {  
    RobotContainer.s_LimelightPort.setIMUMode(2);
    RobotContainer.s_LimelightStbd.setIMUMode(2);

    if (FieldUtils.isRedAlliance()) 
      {robotContainer.getSwerve().resetRotation(robotContainer.getSwerve().getState().Pose.getRotation().plus(Rotation2d.k180deg));}
    else 
      {robotContainer.getSwerve().resetRotation(robotContainer.getSwerve().getState().Pose.getRotation());}
    
    autonomousCommand = robotContainer.getAutoCommand();

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

    if (autonomousCommand != null) 
      {autonomousCommand.cancel();}
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() 
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
