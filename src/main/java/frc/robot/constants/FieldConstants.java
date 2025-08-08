package frc.robot.constants;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.FieldUtils;
import frc.robot.util.AlliancePose2dSup;

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

  public static final AlliancePose2dSup processor = new AlliancePose2dSup(5.575, 0.950, 180);
  public static final AlliancePose2dSup processorOpp = new AlliancePose2dSup(11.873, 7.101, 180);

  public static final AlliancePose2dSup cage1 = new AlliancePose2dSup(7.580, 4.880, 0);
  public static final AlliancePose2dSup cage2 = new AlliancePose2dSup(7.580, 5.969, 0);
  public static final AlliancePose2dSup cage3 = new AlliancePose2dSup(7.580, 7.061, 0);

  public static final AlliancePose2dSup[] leftStations = 
  {
    new AlliancePose2dSup(1.670, 7.376, 36)
  };

  public static final ArrayList<Translation2d> redAlgaeBackoff = new ArrayList<Translation2d>()
  {{
    for (int i = 0; i <= 5; i++)
      {add(redReefMidpoints.get(i).plus(redReefMidpoints.get(i).minus(FieldUtils.GeoFencing.reefRed.getCentre()).times(2)));}
  }};
  
  public static final ArrayList<Translation2d> blueAlgaeBackoff = new ArrayList<Translation2d>()
  {{
    for (int i = 0; i <= 5; i++)
      {add(blueReefMidpoints.get(i).plus(blueReefMidpoints.get(i).minus(FieldUtils.GeoFencing.reefBlue.getCentre()).times(2)));}
  }};
}
