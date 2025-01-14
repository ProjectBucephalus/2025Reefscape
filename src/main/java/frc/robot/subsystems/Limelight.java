// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase 
{  
  private boolean doRejectUpdate;
  public LimelightHelpers.PoseEstimate llPoseEstimator;
  public SwerveDrivePoseEstimator WPIPosEst;
  public Swerve s_Swerve;
  private LimelightHelpers.PoseEstimate mt1;
  private int[] validIDs = Constants.Vision.validIDs;
  
  
  /** Creates a new Limelight. */
  public Limelight(Swerve s_Swerve) 
  {
    llPoseEstimator = new PoseEstimate(null, 0, 0, 0, 0, 0, 0, null, false);
    WPIPosEst = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, new Rotation2d(), Swerve.getModulePositions(), new Pose2d());
    this.s_Swerve = s_Swerve;
  }

  public Pose2d getPose() 
  {
    return WPIPosEst.getEstimatedPosition();
  }

  public boolean getStatus()
  {
    return true;
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.Vision.limeLightName, validIDs);

    mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.limeLightName);

    doRejectUpdate = false;

    SmartDashboard.putNumber("Raw Fiducials Length", LimelightHelpers.getRawFiducials(Constants.Vision.limeLightName).length);
      
    if(mt1 == null)
    {
      doRejectUpdate = true;
      SmartDashboard.putString("LL Error", "mt1 = null");
    }
    else if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
    {
      if(mt1.rawFiducials[0].ambiguity > .9)
      {
        //doRejectUpdate = true;
        SmartDashboard.putString("LL Error", "Ambiguity > .9");
      }
      else if(mt1.rawFiducials[0].distToCamera > 3)
      {
        //doRejectUpdate = true;
        SmartDashboard.putString("LL Error", "Distance to Camera > 3");
      }
    }
    else if(mt1.tagCount == 0)
    {
      //doRejectUpdate = true;
      SmartDashboard.putString("LL Error", "Tag Count = 0");
    }
    else
    {
      doRejectUpdate = false;
      SmartDashboard.putString("LL Error", "clear");

    }

    SmartDashboard.putBoolean("Reject LL Update", doRejectUpdate);
    SmartDashboard.putString("mt1 Pose", mt1.pose.toString());
    
    if(!doRejectUpdate)
    {
      WPIPosEst.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
      WPIPosEst.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
      s_Swerve.setPose(mt1.pose);
    }

    SmartDashboard.putNumber("Robot X", mt1.pose.getX());
    SmartDashboard.putNumber("Robot Y", mt1.pose.getY());
    SmartDashboard.putNumber("Robot Rotation", mt1.pose.getRotation().getDegrees());
    SmartDashboard.putBoolean("I Existance", true);
    SmartDashboard.putNumber("Tag Count", mt1.tagCount);
    SmartDashboard.putNumber("Tag Span", mt1.tagSpan);
    SmartDashboard.putString("MT1", mt1.toString());
    SmartDashboard.putNumber("Avg Tag Area", mt1.avgTagArea);
    SmartDashboard.putNumber("Avg Tag Distance", mt1.avgTagDist);
    SmartDashboard.putNumber("Valid IDs", validIDs.length);
    SmartDashboard.putNumber("Latency", mt1.latency);
    SmartDashboard.putNumber("Timestamp", mt1.timestampSeconds);
   
    /* @TODO: UNCOMMENT WHEN LIMELIGHT 3G ARRIVES */
    /* LimelightHelpers.SetRobotOrientation(Constants.Vision.limelightName, 0, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.limelightName);
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate){
    }

    WPIPosEst.setVisionMeasurementStdDevs (VecBuilder.fill(.7,.7,9999999));
    WPIPosEst.addVisionMeasurement (mt2.pose, mt2.timestampSeconds);

    SmartDashboard.putNumber("Robot X", WPIPosEst.getEstimatedPosition().getX());
    SmartDashboard.getNumber("Robot Y", WPIPosEst.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Robot Rotation", WPIPosEst.getEstimatedPosition().getRotation().getDegrees());
    */
  }
}
