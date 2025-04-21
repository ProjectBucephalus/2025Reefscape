package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class DiffectorGeometry 
{
  public static final double maxRotation = 5;
  /** Maximum total angle the arm is allowed to rotate away from centre */
  public static final double maxAbsAngle = maxRotation * 360;
  /** Above this angle, the arm can turn towards centre even if it's a longer path */
  public static final double turnBackThreshold = 135;
  
  /** Physical upper limit of the elevator, metres above the ground */
  public static final double maxZ = 1.76;
  /** Physical lower limit of the elevator when horizontal, metres above the ground */
  public static final double minZ = 0.36;
  /** Elevation at which all rotations are safe */
  public static final double safeElevation = 0.9;
  public static final double coralFunnelElevation = 1.0;
  public static final double algaeClawElevation = 0.75;
  public static final double reefSafeElevation = 1;
  public static final double algaeSafeElevation = 1;
  public static final double climberClearanceThreshold = 0.7;
  
  /** Arm rotation check tollerance, degrees */
  public static final double angleTolerance = 2;
  /** Angle either side of 0 to consider "vertical" */
  public static final double uprightTolerance = 15;
  /** Angle either side of 180 to consider "vertical" */
  public static final double downsideTolerance = 45;
  /** Elevation height check tolerance, m */
  public static final double elevationTolerance = 0.01;

  public static final int algaeEjectSpeedAngleThreshold = 30;

  /** Manipulator arm point-cloud */
  public static final Translation2d[] armGeometry = new Translation2d[]
  {
    new Translation2d(0.275, 0.295),
    new Translation2d(0.275, 0.445),
    new Translation2d(0.000, 0.490),
    new Translation2d(-0.275, 0.445),
    new Translation2d(-0.275, 0.295),
    new Translation2d(-0.110, -0.545),
    new Translation2d(0.000, -0.555),
    new Translation2d(0.110, -0.545)
  };
  /** Manipulator arm point-cloud when holding Algae */
  public static final Translation2d[] armGeometryAlgae = new Translation2d[]
  {
    new Translation2d(0.275, 0.295),
    new Translation2d(0.275, 0.445),
    new Translation2d(0.000, 0.490),
    new Translation2d(-0.275, 0.445),
    new Translation2d(-0.275, 0.295),
    new Translation2d(0.216, -0.646),
    new Translation2d(0.187, -0.753),
    new Translation2d(0.108, -0.833),
    new Translation2d(0.000, -0.861),
    new Translation2d(-0.108, -0.833),
    new Translation2d(-0.187, -0.753),
    new Translation2d(-0.216, -0.646)
  };

  /* Deck obstruction geometry */
  public static final double deckHeight  = 0.16;
  public static final double railHeight  = 0.2;
  public static final double railLateral = 0.45;
  public static final double railMedial  = 0.37;

  /** For IK, angle the arm is projected to test for immediate collisions, degrees */
  public static final double projectionAngle = 5;
  /** For IK, distance the arm is projected down to test for immediate collisions, m */
  public static final double projectionElevation = 0.05;

  /** For pathfollowing, elevation/rotation "distance" to set the dynamic target position at */
  public static final Translation2d unitTravel = new Translation2d(projectionElevation, projectionAngle);

  public static final double reefSafetyRadius = 1.7;

  /** Distance from centre of barge where arm height needs to be checked, metres */
  public static final double bargeSafetyWidth = 0.85;
  /** Minimum height over ground where arm height needs to be checked, metres */
  public static final double bargeSafetyHeight = 1;
}
