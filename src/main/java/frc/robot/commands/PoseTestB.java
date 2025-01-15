// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Conversions;
import frc.robot.constants.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseTestB extends Command 
{
  public Swerve s_Swerve;
  public Pose2d target;

  /** Creates a new PoseTestB. */
  public PoseTestB(Swerve s_Swerve) 
  {
    SmartDashboard.putString("TestStatus:", "Creating B");
    this.s_Swerve = s_Swerve;
    target = Constants.Testing.poseB;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    SmartDashboard.putString("TestStatus:", "Init B");
    s_Swerve.setTarget(target.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putString("TestStatus:", "Driving to B");
    s_Swerve.driveFenced(Conversions.clamp(target.getX()-s_Swerve.getPose().getX()),Conversions.clamp(target.getY()-s_Swerve.getPose().getY()),0,0,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    SmartDashboard.putString("TestStatus:", "Ending B");
    new WaitCommand(Constants.Testing.delay).andThen(new PoseTestA(s_Swerve));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return (target.getTranslation().getDistance(s_Swerve.getPose().getTranslation()) <= 0.1);
  }
}
