// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.LimelightHelpers;

public class Limelight extends SubsystemBase 
{  
  private boolean doRejectUpdate;
  public SwerveDrivePoseEstimator WPIPosEst;
  public Swerve s_Swerve;
  private LimelightHelpers.PoseEstimate mt2;
  private int[] validIDs = Constants.Vision.validIDs;
  
  /** Creates a new Limelight. */
  public Limelight(Swerve s_Swerve) 
  {
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
    
    LimelightHelpers.SetRobotOrientation(Constants.Vision.limeLightName, s_Swerve.getGyroYaw().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.limeLightName);
 
    doRejectUpdate = false;
    
    if (mt2.tagCount == 0) 
    {
      doRejectUpdate = true;
    }
    
    if(!doRejectUpdate)
    {
      WPIPosEst.setVisionMeasurementStdDevs (VecBuilder.fill(.7,.7,9999999));
      WPIPosEst.addVisionMeasurement (mt2.pose, mt2.timestampSeconds);
    }  

    if(mt2 != null)
    {
      SmartDashboard.putString("mt2 Pose", mt2.pose.toString());
    }
    
    SmartDashboard.putNumber("Robot X", WPIPosEst.getEstimatedPosition().getX());
    SmartDashboard.getNumber("Robot Y", WPIPosEst.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Robot Rotation", WPIPosEst.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putBoolean("Existance", true);
    SmartDashboard.putString("MegaTag2 BotPose", WPIPosEst.toString());
  }
}
