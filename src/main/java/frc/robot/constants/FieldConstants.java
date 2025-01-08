package frc.robot.constants;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {

    public static final double FIELD_LENGTH = 16.54;
    public static final double FIELD_WIDTH = 8.21;

    public static boolean isRedAlliance() 
    {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public static Pose2d flipPose(Pose2d pose) 
    {
        // flip pose when red
        if (isRedAlliance()) {
            Rotation2d rot = pose.getRotation();
            // reflect the pose over center line, flip both the X and the rotation
            return new Pose2d(FIELD_LENGTH - pose.getX(), pose.getY(), new Rotation2d(-rot.getCos(), rot.getSin()));
        }

        // Blue or we don't know; return the original pose
        return pose;
    }

    public static Translation2d flipTranslation(Translation2d position) 
    {
        // flip when red
        if (isRedAlliance()) {
            // reflect the pose over center line, flip both the X
            return new Translation2d(FIELD_LENGTH - position.getX(), position.getY());
        }

        // Blue or we don't know; return the original position
        return position;
    }

    public static Pose2d transformToPose2d(Transform2d position) 
    {
        // Converts a Transform2d to a pose.
        return new Pose2d(position.getTranslation(), position.getRotation());
    }

    public static Pose2d translationToPose2d(Translation2d position) 
    {
        // Converts a Transform2d to a pose.
        return new Pose2d(position.getX(), position.getY(), position.getAngle());
    }

    public static PathPlannerPath loadPath(String pathName) 
    {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return path;
        } catch (Exception e) {
            DriverStation.reportError(String.format("Unable to load path: %s", pathName), true);
        }
        return null;
    }
}