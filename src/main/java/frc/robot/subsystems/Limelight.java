// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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

  private SwerveDriveState driveState;
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

    objectRota = s_Swerve.getState().Pose.getRotation();
    //FieldObject2d algaeTest = (s_Swerve.field.getObject("Algae"));
    /* FieldObject2d test = (s_Swerve.field.getObject("Box"));
    test.setPoses(new Pose2d(2.65,4.15,new Rotation2d()), new Pose2d(8.25,4.15,new Rotation2d(0)), new Pose2d(1,4.15,new Rotation2d()), new Pose2d(2,2,new Rotation2d())); 
    SmartDashboard.putString("Test Box", test.toString()); */

    objectTest = (s_Swerve.field.getObject("Test"));
    
    SmartDashboard.putBoolean("Use Limelight", true);
  }
  
  public Pose2d getPose() 
  {return mt2.pose;}

  public Pose3d getObjectPose()
  {return LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);}
   
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    
    //LimelightHelpers.SetRobotOrientation(limelightName, s_Swerve.getPigeon2().getYaw().getValueAsDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
    
    driveState = s_Swerve.getState();
    headingDeg = driveState.Pose.getRotation().getDegrees();
    omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    objectTest.setPose(objectX, objectY, objectRota);
    objectX = getObjectPose().getX();
    objectY = getObjectPose().getY();
    
    if (SmartDashboard.getBoolean("Use Limelight", true)) 
    {
      LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
      LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
      useUpdate = !(mt2 == null || mt2.tagCount == 0 || omegaRps > 2.0);
      SmartDashboard.putBoolean("Use " + limelightName + " update", useUpdate);
      
      if (useUpdate) 
        {s_Swerve.addVisionMeasurement(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));}
    }
    
    if(mt2 != null)
    {
      SmartDashboard.putString(limelightName + " mt2 Pose", mt2.pose.toString());
      SmartDashboard.putNumber(limelightName + " Tag Count", mt2.tagCount);
      SmartDashboard.putString(limelightName + "GetObject Output", getObjectPose().toString());
    }
    SmartDashboard.putNumber(limelightName + "RPS", omegaRps);
  }
}
