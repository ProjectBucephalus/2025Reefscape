package frc.robot.util;
import java.util.ArrayList;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;
import frc.robot.util.GeoFenceObject.ObjectTypes;

public class FieldUtils 
{
  public static final double fieldLength = 17.548;
  public static final double fieldWidth = 8.051;

  public static boolean isRedAlliance() 
  {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  public static Pose2d flipPose(Pose2d pose) 
  {
    // flip pose when red
    if (FieldUtils.isRedAlliance()) 
    {
      Rotation2d rot = pose.getRotation();
      // reflect the pose over center line, flip both the X and the rotation
      return new Pose2d(FieldUtils.fieldLength - pose.getX(), pose.getY(), new Rotation2d(-rot.getCos(), rot.getSin()));
    }

    // Blue or we don't know; return the original pose
    return pose;
  }

  public static int getNearestReefFace(Translation2d robotPos)
  {
    int nearestReefFace;
    ArrayList<Translation2d> localList;

    if (FieldUtils.isRedAlliance()) 
    {   
      localList = Constants.Auto.reefRedMidPoints;
    }
    else
    {
      localList = Constants.Auto.reefBlueMidPoints;
    }

    nearestReefFace = localList.indexOf(robotPos.nearest(localList)); 

    nearestReefFace = Conversions.wrap(nearestReefFace, 1, 6);
    
    return nearestReefFace;
  }

  public static PathPlannerPath loadPath(String pathName) 
  {
    try 
    {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return path;
    } 
    catch (Exception e) 
    {
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
    public static final double fieldNorth = FieldUtils.fieldWidth;

    /** Metres the robot can travel right */
    public static final double fieldSouth = 0;

    /** Metres the robot can travel forwards */
    public static final double fieldEast = FieldUtils.fieldLength;

    /** Metres the robot can travel back */
    public static final double fieldWest = 0;

    /** Buffer zone for the field walls in metres */
    public static final double wallBuffer = 0.5;
    
    /** Inscribed diameter of the reef hexagon (i.e. distance between opposite faces) in metres */
    public static final double inscribedReefDiameter = 1.663;
    /** Circumscribed diameter of the reef hexagon (i.e. distance between opposite points) in metres */
    public static final double circumscribedReefDiameter = 1.920;
    /** Circumscribed diameter of the reef zone hexagon (i.e. distance between opposite points) in metres */
    public static final double circumscribedReefZoneDiameter = 2.742;
    
    /** Buffer zone for the reef in metres */
    public static final double reefBuffer = 0.5;

    /** Buffer zone for the barge zone in metres */
    public static final double bargeBuffer = 0.5;

    public static final double cornerWidth  = 1.276;
    public static final double cornerLength = 1.758;

    public static final GeoFenceObject field = new GeoFenceObject
    (
      GeoFencing.fieldWest, 
      GeoFencing.fieldSouth, 
      GeoFencing.fieldEast, 
      GeoFencing.fieldNorth, 
      GeoFencing.wallBuffer,
      0.0,
      ObjectTypes.walls
    );

    public static final GeoFenceObject reefBlue      = new GeoFenceObject(4.489, 4.026, GeoFencing.reefBuffer, GeoFencing.circumscribedReefDiameter / 2, 0, 6);
    public static final GeoFenceObject reefZoneBlue  = new GeoFenceObject(4.489, 4.026, GeoFencing.reefBuffer, GeoFencing.circumscribedReefZoneDiameter / 2, 0, 6);
    public static final GeoFenceObject reefRed       = new GeoFenceObject(13.059, 4.026, GeoFencing.reefBuffer, GeoFencing.circumscribedReefDiameter / 2, 180, 6);
    public static final GeoFenceObject reefZoneRed   = new GeoFenceObject(13.059, 4.026, GeoFencing.reefBuffer, GeoFencing.circumscribedReefZoneDiameter / 2, 180, 6);
    public static final GeoFenceObject bargeColumn   = new GeoFenceObject(8.774, 4.026, 0.25, 0.15);
    public static final GeoFenceObject bargeZoneBlue = new GeoFenceObject(8.190, 4.331, 9.358, FieldUtils.fieldWidth, GeoFencing.bargeBuffer, 0.1, ObjectTypes.box);
    public static final GeoFenceObject bargeZoneRed  = new GeoFenceObject(8.190, 3.721, 9.358, 0, GeoFencing.bargeBuffer, 0.1, ObjectTypes.box);
    public static final GeoFenceObject cornerSBlue   = new GeoFenceObject(GeoFencing.fieldWest, GeoFencing.fieldSouth + GeoFencing.cornerWidth, GeoFencing.fieldWest + GeoFencing.cornerLength, GeoFencing.fieldSouth, GeoFencing.wallBuffer);
    public static final GeoFenceObject cornerNBlue   = new GeoFenceObject(GeoFencing.fieldWest, GeoFencing.fieldNorth - GeoFencing.cornerWidth, GeoFencing.fieldWest + GeoFencing.cornerLength, GeoFencing.fieldNorth, GeoFencing.wallBuffer);
    public static final GeoFenceObject cornerSRed    = new GeoFenceObject(GeoFencing.fieldEast, GeoFencing.fieldSouth + GeoFencing.cornerWidth, GeoFencing.fieldEast - GeoFencing.cornerLength, GeoFencing.fieldSouth, GeoFencing.wallBuffer);
    public static final GeoFenceObject cornerNRed    = new GeoFenceObject(GeoFencing.fieldEast, GeoFencing.fieldNorth - GeoFencing.cornerWidth, GeoFencing.fieldEast - GeoFencing.cornerLength, GeoFencing.fieldNorth, GeoFencing.wallBuffer);

    public static final GeoFenceObject[] fieldBlueGeoFence = 
    {
      GeoFencing.field, 
      GeoFencing.reefBlue, 
      GeoFencing.reefZoneRed, 
      GeoFencing.bargeColumn, 
      GeoFencing.bargeZoneRed, 
      GeoFencing.cornerSBlue, 
      GeoFencing.cornerNBlue, 
      GeoFencing.cornerSRed, 
      GeoFencing.cornerNRed
    };

    public static final GeoFenceObject[] fieldRedGeoFence = 
    {
      GeoFencing.field, 
      GeoFencing.reefRed, 
      GeoFencing.reefZoneBlue, 
      GeoFencing.bargeColumn, 
      GeoFencing.bargeZoneBlue, 
      GeoFencing.cornerSBlue, 
      GeoFencing.cornerNBlue, 
      GeoFencing.cornerSRed, 
      GeoFencing.cornerNRed
    };
    
    /** Radius from robot centre in metres where geofence is triggered */
    public static final double robotRadiusInscribed = 0.45;
    /** Radius from robot centre in metres where geofence is triggered */
    public static final double robotRadiusCircumscribed = 0.55;
    /** Speed threshold at which the robot changes between radii, in meters per second*/
    public static final double robotSpeedThreshold = 1.5;
  }
}