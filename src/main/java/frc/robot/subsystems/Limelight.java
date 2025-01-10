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
import frc.robot.util.LimelightHelpers.PoseEstimate;


public class Limelight extends SubsystemBase 
{  
  boolean doRejectUpdate;
  public LimelightHelpers.PoseEstimate llPoseEstimator;
  public SwerveDrivePoseEstimator WPIPosEst;
  
  
  /** Creates a new Limelight. */
  public Limelight() 
  {
    llPoseEstimator = new PoseEstimate(null, 0, 0, 0, 0, 0, 0, null, false);
    WPIPosEst = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, new Rotation2d(), Swerve.getModulePositions(), new Pose2d());
  }

  public boolean getStatus()
  {
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int[] validIDs = {17, 18, 19, 20, 21, 22};
    LimelightHelpers.SetFiducialIDFiltersOverride("Limelight", validIDs);

    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
    if(mt1 == null)
      {doRejectUpdate = true;}
    else if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
    {
      if(mt1.rawFiducials[0].ambiguity > .7)
        {doRejectUpdate = true;}

      else if(mt1.rawFiducials[0].distToCamera > 3)
        {doRejectUpdate = true;}
    }
    else if(mt1.tagCount == 0)
      {doRejectUpdate = true;}
    else
      {doRejectUpdate = false;}


    if(!doRejectUpdate)
    {
      WPIPosEst.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
      WPIPosEst.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
    }

    SmartDashboard.putNumber("Robot X", WPIPosEst.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Robot Y", WPIPosEst.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Robot Rotation", WPIPosEst.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putBoolean("I Existance", true);

    /* @TODO: UNCOMMENT WHEN LIMELIGHT 3G ARRIVES */
    /* LimelightHelpers.SetRobotOrientation("Limelight", 0, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("Limelight");
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
