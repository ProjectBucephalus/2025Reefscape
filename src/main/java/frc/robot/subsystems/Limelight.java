// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.LimelightHelpers;

public class Limelight extends SubsystemBase 
{  
  private boolean useUpdate;
  public CommandSwerveDrivetrain s_Swerve;
  private LimelightHelpers.PoseEstimate mt2;
  private int[] validIDs = Constants.Vision.validIDs;

  private double headingDeg;
  private double omegaRps;
  private double objectX;
  private double objectY;
  FieldObject2d objectTest;
  private Rotation2d objectRota;

  private final String limelightName;
  
  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain s_Swerve, String name) 
  {
    this.s_Swerve = s_Swerve;
    limelightName = name;

    objectTest = (s_Swerve.field.getObject("Test"));
    
    SmartDashboard.putBoolean("Use Limelight", true);
  }
  
  public Pose2d getPose() 
  {return mt2.pose;}
  
  public Pose3d getObjectPose()
  {return LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);}

  public void setIMUMode(int mode)
  {LimelightHelpers.SetIMUMode(limelightName, mode);}
   
  @Override
  public void periodic() 
  { 

    headingDeg = s_Swerve.getPigeon2().getYaw().getValueAsDouble();
    omegaRps = Units.radiansToRotations(s_Swerve.getState().Speeds.omegaRadiansPerSecond);
    objectRota = s_Swerve.getState().Pose.getRotation();
    
    
    objectTest.setPose(objectX, objectY, objectRota);
    
    
    if (SmartDashboard.getBoolean("Use Limelight", true))
    {
      LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
      LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
      
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
      
      objectX = (getObjectPose().getX() + mt2.pose.getX());
      objectY = (getObjectPose().getY() + mt2.pose.getY());
      
      
      useUpdate = !(mt2 == null || mt2.tagCount == 0 || omegaRps > 2.0);
      SmartDashboard.putBoolean("Use " + limelightName + " update", useUpdate);
      
      if (useUpdate) 
      {
        s_Swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        s_Swerve.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
      }
    }
    
    if(mt2 != null)
    {
      SmartDashboard.putString(limelightName + " mt2 Pose", mt2.pose.toString());
    }
    SmartDashboard.putNumber(limelightName + "RPS", omegaRps);
    SmartDashboard.putNumber("Gyro yaw", headingDeg);
  }
}
