// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.util.Conversions;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.SD;

public class Limelight extends SubsystemBase 
{  
  private boolean useUpdate;
  private LimelightHelpers.PoseEstimate mt2;
  private static int[] validIDs = Constants.Vision.reefIDs;
  private LimelightHelpers.PoseEstimate mt1;
  
  private double headingDeg;
  private double omegaRps;
  private double stdDevFactor;
  private double linearStdDev;
  private double rotStdDev;
  
  private final String limelightName;

  private int pipelineIndex = 0;

  public enum TagPOI 
  {
    REEF,
    BARGE,
    PROCESSOR,
    CORALSTATION
  }
  
  /** Creates a new Limelight. */
  public Limelight(String name) 
  {
    limelightName = name;

    SD.IO_LL.init();
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

  public void setThrottle(int throttle)
  {
    NetworkTableInstance.getDefault().getTable(limelightName).getEntry("<throttle_set>").setNumber(throttle);
  }

  public static void setActivePOI(TagPOI activePOI) 
  {
    switch (activePOI) 
    {
      default:
      case REEF:
        validIDs = Constants.Vision.reefIDs;
        break;
      case BARGE:
        validIDs = Constants.Vision.bargeIDs;
        break;
      case PROCESSOR:
      case CORALSTATION:
        validIDs = Constants.Vision.humanPlayerStationIDs;
        break;
    }
  }

  public int updateLimelightPipeline()
  {
    if (SD.IO_LL_EXPOSURE_UP.get())
    {
      SD.IO_LL_EXPOSURE_UP.put(false);
      SD.IO_LL_EXPOSURE.put(Conversions.clamp(SD.IO_LL_EXPOSURE.get().intValue() + 1, 0, 7));
    }
    if (SD.IO_LL_EXPOSURE_DOWN.get())
    {
      SD.IO_LL_EXPOSURE_DOWN.put(false);
      SD.IO_LL_EXPOSURE.put(Conversions.clamp(SD.IO_LL_EXPOSURE.get().intValue() - 1, 0, 7));
    }
    return SD.IO_LL_EXPOSURE.get().intValue();
  }

  @Override
  public void periodic() 
  { 
    if (updateLimelightPipeline() != pipelineIndex)
    {
      pipelineIndex = updateLimelightPipeline();
      LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }

    headingDeg = RobotContainer.s_Swerve.getPigeon2().getYaw().getValueAsDouble();
    omegaRps = Units.radiansToRotations(RobotContainer.swerveState.Speeds.omegaRadiansPerSecond);
    
    LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
    
    LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
    
    if (SD.IO_LL.get())
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

    SD.SENSOR_GYRO.put(headingDeg);
    if (!getLimelightRotation().equals(Rotation2d.kZero))
    SmartDashboard.putNumber("Pose " + limelightName + " Estimate", getLimelightRotation().getDegrees());
    else SmartDashboard.putNumber("Pose " + limelightName + " Estimate", 0);

    SmartDashboard.putNumber(limelightName + " HW Metrics", NetworkTableInstance.getDefault().getTable(limelightName).getEntry("hw").getDouble(0));
  }
}
