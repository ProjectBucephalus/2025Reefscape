// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.util.LimelightHelpers;

public class Limelight extends SubsystemBase 
{  
  private boolean useUpdate;
  private LimelightHelpers.PoseEstimate mt2;
  private int[] validIDs = Constants.Vision.validIDs;

  private double headingDeg;
  private double omegaRps;

  private final String limelightName;
  
  /** Creates a new Limelight. */
  public Limelight(String name) 
  {
    limelightName = name;
    
    SmartDashboard.putBoolean("Use Limelight", false);
  }

  public void setIMUMode(int mode)
    {LimelightHelpers.SetIMUMode(limelightName, mode);}
   
  @Override
  public void periodic() 
  { 
    headingDeg = RobotContainer.s_Swerve.getPigeon2().getYaw().getValueAsDouble();
    omegaRps = Units.radiansToRotations(RobotContainer.state.Speeds.omegaRadiansPerSecond);
    
    if (SmartDashboard.getBoolean("Use Limelight", false))
    {
      LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
      
      LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
      
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
      
      useUpdate = !(mt2 == null || mt2.tagCount == 0 || omegaRps > 2.0);
      SmartDashboard.putBoolean("Use " + limelightName + " update", useUpdate);
      
      if (useUpdate) 
      {
        RobotContainer.s_Swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        RobotContainer.s_Swerve.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
      }
    }

    SmartDashboard.putNumber("Gyro yaw", headingDeg);
  }
}
