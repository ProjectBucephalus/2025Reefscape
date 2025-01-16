// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Conversions;
import frc.robot.constants.Constants;

public class PoseTest extends Command 
{
  public Swerve s_Swerve;
  public Pose2d target;
  private boolean A;

  /** Creates a new PoseTestA. */
  public PoseTest(Swerve s_Swerve) 
  {
    SmartDashboard.putString("TestStatus:", "Creating A");
    this.s_Swerve = s_Swerve;
    target = Constants.Testing.poseA;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    SmartDashboard.putString("TestStatus:", "Init A");
    s_Swerve.setTarget(target.getRotation().getDegrees());
    A = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putString("TestStatus:", "Driving to A");
    s_Swerve.driveFenced(Conversions.clamp(3*(target.getX()-s_Swerve.getPose().getX())),Conversions.clamp(3*(target.getY()-s_Swerve.getPose().getY())),0,0,true);
    if (target.getTranslation().getDistance(s_Swerve.getPose().getTranslation()) <= 0.1)
    {
      if(A)
      {
        target = Constants.Testing.poseB;
        s_Swerve.setTarget(target.getRotation().getDegrees());
        A = false;
      }
      else
      {
        target = Constants.Testing.poseA;
        s_Swerve.setTarget(target.getRotation().getDegrees());
        A = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    SmartDashboard.putString("TestStatus:", "Ending A");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    
    return false;
  }
}
