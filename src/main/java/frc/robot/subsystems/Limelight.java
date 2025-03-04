// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase 
{  
  private boolean useUpdate;
  private LimelightHelpers.PoseEstimate mt2;
  private int[] validIDs = Constants.Vision.validIDs;
  private LimelightHelpers.PoseEstimate mt1;

  
  private double headingDeg;
  private double omegaRps;
  private double stdDevFactor;
  private double linearStdDev;
  private double rotStdDev;
  
  private final String limelightName;
  
  /** Creates a new Limelight. */
  public Limelight(String name) 
  {
    limelightName = name;

    SmartDashboard.putBoolean("Use Limelight", false);
  }

  public void setIMUMode(int mode)
    {LimelightHelpers.SetIMUMode(limelightName, mode);}

  public Rotation2d getLimelightRotation()
  {
    mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (mt1 != null)
      {return mt1.pose.getRotation();}
    return Rotation2d.kZero;
  }
   
  @Override
  public void periodic() 
  { 
    headingDeg = RobotContainer.s_Swerve.getPigeon2().getYaw().getValueAsDouble();
    omegaRps = Units.radiansToRotations(RobotContainer.swerveState.Speeds.omegaRadiansPerSecond);
    
    LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
    
    LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
    
    if (SmartDashboard.getBoolean("Use Limelight", false))
    {
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
      
      useUpdate = !(mt2 == null || mt2.tagCount == 0 || omegaRps > 2.0);
      SmartDashboard.putBoolean("Use " + limelightName + " update", useUpdate);
      
      if (useUpdate) 
      {
        stdDevFactor = Math.pow(mt2.avgTagDist, 2.0) / mt2.tagCount;

        linearStdDev = Constants.Vision.linearStdDevBaseline * stdDevFactor;
        rotStdDev = Constants.Vision.rotStdDevBaseline * stdDevFactor;

        RobotContainer.s_Swerve.setVisionMeasurementStdDevs(VecBuilder.fill(linearStdDev, linearStdDev, rotStdDev));
        RobotContainer.s_Swerve.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
      }
    }

    SmartDashboard.putNumber("Gyro yaw", headingDeg);
    if (!getLimelightRotation().equals(Rotation2d.kZero))
    SmartDashboard.putNumber("Pose " + limelightName + " Estimate", getLimelightRotation().getDegrees());
    else SmartDashboard.putNumber("Pose " + limelightName + " Estimate", 0);
  }
}
