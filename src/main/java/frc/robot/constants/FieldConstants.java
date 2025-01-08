package frc.robot.constants;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.GeoFenceObject;
import frc.robot.util.GeoFenceObject.ObjectTypes;

public class FieldConstants 
{
    public static final double fieldLength = 17.55;
    public static final double fieldWidth = 8.05;

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
            return new Pose2d(fieldLength - pose.getX(), pose.getY(), new Rotation2d(-rot.getCos(), rot.getSin()));
        }

        // Blue or we don't know; return the original pose
        return pose;
    }

    public static Translation2d flipTranslation(Translation2d position) 
    {
        // flip when red
        if (isRedAlliance()) {
            // reflect the pose over center line, flip both the X
            return new Translation2d(fieldLength - position.getX(), position.getY());
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

    public final class GeoFencing
    {   
        // Relative to the centre of the robot, in direction the robot is facing
        // These values are the distance in metres to the virtual wall the robot will stop at
        // 0 means the wall is running through the middle of the robot
        // negative distances will have the robot start outside the area, and can only move into it
        /** Metres the robot can travel left */
        public static final double fieldLeft = fieldWidth;

        /** Metres the robot can travel right */
        public static final double fieldRight = 0;

        /** Metres the robot can travel forwards */
        public static final double fieldFront = fieldLength;

        /** Metres the robot can travel back */
        public static final double fieldBack = 0;    

        public static final GeoFenceObject field = new GeoFenceObject
        (
            -fieldBack, 
            -fieldRight, 
            fieldFront, 
            fieldLeft, 
            0.5,
            0.0,
            ObjectTypes.walls
        );

        public static final double inscribedReefDiameter = 1.663;

        public static final GeoFenceObject reefBlue = new GeoFenceObject(4.489, 4.026, 0, inscribedReefDiameter / 2);
        public static final GeoFenceObject reefRed = new GeoFenceObject();
        public static final GeoFenceObject bargeColumn = new GeoFenceObject();
        public static final GeoFenceObject bargeZoneBlue = new GeoFenceObject();
        public static final GeoFenceObject bargeZoneRed = new GeoFenceObject();
        public static final GeoFenceObject cornerSBlue = new GeoFenceObject();
        public static final GeoFenceObject cornerNBlue = new GeoFenceObject();
        public static final GeoFenceObject cornerSRed = new GeoFenceObject();
        public static final GeoFenceObject cornerNRed = new GeoFenceObject();

        public static final GeoFenceObject[] fieldGeoFence = {field};
        
        /** Radius from robot centre in metres where geofence is triggered */
        public static final double robotRadius = 0.55;
    }
}