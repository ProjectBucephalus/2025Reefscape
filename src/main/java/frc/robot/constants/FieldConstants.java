package frc.robot.constants;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.FieldUtils;

public class FieldConstants 
{
  public static final ArrayList<Translation2d> blueReefMidpoints = FieldUtils.GeoFencing.reefBlue.getMidPoints();
  public static final ArrayList<Translation2d> redReefMidpoints = FieldUtils.GeoFencing.reefRed.getMidPoints();

  public static final ArrayList<Translation2d> blueBargePoints = new ArrayList<Translation2d>()
  {{
    add(new Translation2d(FieldUtils.fieldLength / 2, 7.261));
    add(new Translation2d(FieldUtils.fieldLength / 2, 6.615));
    add(new Translation2d(FieldUtils.fieldLength / 2, 6.169));
    add(new Translation2d(FieldUtils.fieldLength / 2, 5.6245));
    add(new Translation2d(FieldUtils.fieldLength / 2, 5.08));
  }};

  public static final ArrayList<Translation2d> redBargePoints = new ArrayList<Translation2d>(blueBargePoints)
  {{
    forEach(point -> point.rotateAround(FieldUtils.fieldCentre, Rotation2d.k180deg));
  }};

  public static final ArrayList<Translation2d> blueClimbLineups = new ArrayList<Translation2d>()
  {{
    add(new Translation2d(FieldUtils.fieldLength / 2, 7.261));
    add(new Translation2d(FieldUtils.fieldLength / 2, 6.169));
    add(new Translation2d(FieldUtils.fieldLength / 2, 5.08));
  }};

  public static final ArrayList<Translation2d> redClimbLineups = new ArrayList<Translation2d>(blueClimbLineups)
  {{
    forEach(point -> point.rotateAround(FieldUtils.fieldCentre, Rotation2d.k180deg));
  }};

  public static final double coralStationRange = 0.6;

  public static final double bargeWarningRange = 0.6;
}
