// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.constants.DiffectorGeometry;
import frc.robot.constants.Constants.DiffectorConstants.Presets;

/** Add your docs here. */
public class ArmCalculator 
{
  private double minElevation;
  private double maxElevation;
  private double safeElevation;
  private double coralClawElevation;
  private double algaeClawElevation;
  private double uprightTolerance;
  private double downsideTolerance;
  private double projectionAngle;
  private double projectionElevation;

  private double deckHeight;

  private double offset;
  private double maxAbsPos;
  private double reverseOffset;
  private double turnBackThreshold;

  /** Unrotated virtual arm */
  private final Translation2d[] armGeometry;
  /** Unrotated virtual arm when holding Algae*/
  private final Translation2d[] armGeometryAlgae;
  
  public ArmCalculator()
  {
    maxElevation  = DiffectorGeometry.maxZ;
    minElevation  = DiffectorGeometry.minZ;
    safeElevation = DiffectorGeometry.safeElevation;
    coralClawElevation = DiffectorGeometry.coralFunnelElevation;
    algaeClawElevation = DiffectorGeometry.algaeClawElevation;
    uprightTolerance = DiffectorGeometry.uprightTolerance;
    downsideTolerance = DiffectorGeometry.downsideTolerance;
    projectionAngle = DiffectorGeometry.projectionAngle;
    projectionElevation = DiffectorGeometry.projectionElevation;

    maxAbsPos = DiffectorGeometry.maxAbsAngle;
    turnBackThreshold = DiffectorGeometry.turnBackThreshold;
    
    deckHeight    = DiffectorGeometry.deckHeight;

    armGeometry = DiffectorGeometry.armGeometry;
    armGeometryAlgae = DiffectorGeometry.armGeometryAlgae;
  }

  /**
   * Calculates desired path for arm to follow: Ensures the arm is at a safe height, rotates to target, elevates to target
   * @param targetPosition target height/rotation of elevator carriage, (metres above deck)/(degrees total anticlockwise)
   * @param startPosition initial height/rotation of elevator carriage, (metres above deck)/(degrees total anticlockwise)
   * @return Translation2d array containing the projected path
   */
  public ArrayList<Translation2d> pathfindArm(Translation2d targetPosition, Translation2d startPosition)
  {
    ArrayList<Translation2d> pathOutput = new ArrayList<Translation2d>();

    Translation2d relativeTarget = new Translation2d(targetPosition.getX(), Conversions.mod(targetPosition.getY(), 360));

    Translation2d robotPos = RobotContainer.swerveState.Pose.getTranslation();

    GeoFenceObject allianceReef = FieldUtils.isRedAlliance() ? FieldUtils.GeoFencing.reefRed : FieldUtils.GeoFencing.reefBlue;

    if (RobotContainer.algae)
      {safeElevation = DiffectorGeometry.algaeSafeElevation;}
    else if (robotPos.getDistance(allianceReef.getCentre()) <= DiffectorGeometry.reefSafetyRadius) 
      {safeElevation = DiffectorGeometry.reefSafeElevation;}
    else
      {safeElevation = DiffectorGeometry.safeElevation;}

    // Certain positions put the arm lower than it would otherwise be allowed to go
    if (Presets.lowDiffectorPositions.stream().anyMatch(relativeTarget::equals))
    { // Forced safe path for unsafe targets
      pathOutput.add(new Translation2d(Math.max(safeElevation, startPosition.getX()), startPosition.getY()));
      pathOutput.add(new Translation2d(Math.max(safeElevation, startPosition.getX()), targetPosition.getY()));
      pathOutput.add(new Translation2d(safeElevation, targetPosition.getY())); // Ensuring arm is not rotating
      pathOutput.add(targetPosition);

      return pathOutput;
    }

    if (Presets.highDiffectorPositions.stream().anyMatch(relativeTarget::equals))
    { // Forced safe path for high scoring positions
      pathOutput.add(new Translation2d(Math.max(safeElevation, startPosition.getX()), startPosition.getY()));
      if (MathUtil.isNear(relativeTarget.getY(), 180, 90))
      {
        pathOutput.add(new Translation2d(Math.max(safeElevation, startPosition.getX()), goShortest(180, targetPosition.getY())));
        pathOutput.add(new Translation2d(targetPosition.getX(), goShortest(180, targetPosition.getY())));
      }
      else
      {
        pathOutput.add(new Translation2d(Math.max(safeElevation, startPosition.getX()), goShortest(0, targetPosition.getY())));
        pathOutput.add(new Translation2d(targetPosition.getX(), goShortest(0, targetPosition.getY())));
      }
      pathOutput.add(targetPosition);

      return pathOutput;
    }
    
    
    // Any other position should be made safe
    targetPosition = new Translation2d(checkPosition(targetPosition), targetPosition.getY());
    
    // Path of arm starts above safe limits, path is safe as given
    if (startPosition.getX() >= safeElevation)
    { // Go to target rotation
      pathOutput.add(new Translation2d(startPosition.getX(), targetPosition.getY()));
      // Go to target posititon
      pathOutput.add(targetPosition);
      return pathOutput;
    }

    double angleChange = targetPosition.getY() - startPosition.getY();
    double angleRelative = Conversions.mod(startPosition.getY(), 360);

    // Elevation change only
    if (Math.abs(angleChange) <= DiffectorGeometry.angleTolerance)
    {
      pathOutput.add(targetPosition);
      return pathOutput;
    }
    
    // Any rotation taking the arm past vertical:
    if
    ( // If goes past both uprights
      Math.abs(angleChange) >= 360 ||
      (angleRelative < 180 + downsideTolerance && angleRelative + angleChange >= 360 - uprightTolerance ) ||
      (angleRelative > 180 - downsideTolerance && angleRelative + angleChange <=   0 + uprightTolerance ) ||
      (angleRelative <   0 + uprightTolerance  && angleRelative + angleChange >= 180 - downsideTolerance) ||
      (angleRelative > 180 + downsideTolerance && angleRelative + angleChange >= 540 - downsideTolerance) ||
      (angleRelative > 360 - uprightTolerance  && angleRelative + angleChange <= 180 + downsideTolerance) ||
      (angleRelative < 180 - downsideTolerance && angleRelative + angleChange <=-180 + downsideTolerance) ||
      (angleRelative < 180 - downsideTolerance && angleRelative + angleChange >= 360 - uprightTolerance ) ||
      (angleRelative > 180 + downsideTolerance && angleRelative + angleChange <=   0 + uprightTolerance )
    )
    {
      // Intermediate waypoint: Safe elevation at initial rotation
      pathOutput.add(new Translation2d(Math.max(startPosition.getX(), safeElevation), startPosition.getY()));
      pathOutput.add(new Translation2d(Math.max(startPosition.getX(), safeElevation), targetPosition.getY()));
    }
    
    else if
    ( // Starts upright
      (angleRelative > 360 - uprightTolerance || angleRelative < 0 + uprightTolerance) ||
      // Anticlockwise angle change goes past upright
      angleRelative + angleChange >= 360 - uprightTolerance || 
      // Clockwise angle change goes past upright
      angleRelative + angleChange <= 0 + uprightTolerance
    )
    {
      // Intermediate waypoint: Safe elevation at initial rotation
      pathOutput.add(new Translation2d(Math.max(algaeClawElevation, startPosition.getX()), startPosition.getY()));
      pathOutput.add(new Translation2d(Math.max(algaeClawElevation, startPosition.getX()), targetPosition.getY()));
    }

    else if
    ( // Starts upside-down
      (angleRelative > 180 - downsideTolerance && angleRelative < 180 + downsideTolerance) ||
      // Anticlockwise angle change goes past upside-down
      (angleRelative < 180 + downsideTolerance && angleRelative + angleChange >= 180 - downsideTolerance) ||
      // Clockwise angle change goes past upside-down
      (angleRelative > 180 - downsideTolerance && angleRelative + angleChange <= 180 + downsideTolerance)
    )
    {
      // Intermediate waypoint: Safe elevation at initial rotation
      pathOutput.add(new Translation2d(Math.max(coralClawElevation, startPosition.getX()), startPosition.getY()));
      pathOutput.add(new Translation2d(Math.max(coralClawElevation, startPosition.getX()), targetPosition.getY()));
    }


    // Rotation does not go past vertical -> never needs to go higher than start or end
    else if (startPosition.getX() < checkAngle(targetPosition.getY())) // Start is lower than is safe for final rotation
    { // Go to safe elevation for final rotation, then rotate
      pathOutput.add(new Translation2d(checkAngle(targetPosition.getY()), startPosition.getY()));
      pathOutput.add(new Translation2d(checkAngle(targetPosition.getY()), targetPosition.getY()));
    }

    /* Arm starts lower than is safe
    else if (startPosition.getX() <= checkAngle(startPosition.getY()))
    { // Ensure the arm is safe before moving from vertical
      pathOutput.add(new Translation2d(safeElevation, startPosition.getY()));
      pathOutput.add(new Translation2d(safeElevation, targetPosition.getY()));
    }*/

    else // Start is high enough to rotate to final rotation
      {pathOutput.add(new Translation2d(startPosition.getX(), targetPosition.getY()));}


    // Add Target waypoint:
    pathOutput.add(targetPosition);

    return pathOutput;
  }

  /**
   * Adjusts the elevation of a given position to keep above safe limits
   * @param elevation the intended elevation
   * @param currentAngle the angle of the arm to check
   * @return maximum of the intended elevation and the safe elevation for the given angle
   */
  public double checkPosition(Translation2d position)
    {return MathUtil.clamp(position.getX(), checkAngle(position.getY()), maxElevation);}

  /**
   * Returns the minimum safe arm height for a given angle
   * @param angle the angle of the arm to check
   */
  public double checkAngle(double angle)
  {
    /*
     *  NOTE:
     *    Right-handed rotation on the +Y (forwards) robot axis, +Rotation called Anticlockwise
     *    Arm-relative geometry uses X/Y, mapping to Robot-relative X/Z
     *    +X = Port (robot Left), -X = Starboard (robot Right)
     *    Topside of electronics = Deck, Obstructing mechanisms/bumbers = Rail
     *    Centreline = Mast, Near = Medial, Far = Lateral
     */
    
    // Rotation value of the input angle
    Rotation2d rotation = new Rotation2d(Units.degreesToRadians(angle));
    // Temporary, rotated reference point
    Translation2d geometryPointRotated;
    // Running value of the lowest point relative to the deck/rail
    double lowestPoint = 0;
    
    if (RobotContainer.algae)
    {
      for (Translation2d geometryPoint : armGeometryAlgae)
      {
        geometryPointRotated = geometryPoint.rotateBy(rotation);
        lowestPoint = Math.min(lowestPoint, geometryPointRotated.getY() - deckHeight);
      }
    }
    else
    {
      for (Translation2d geometryPoint : armGeometry)
      {
        geometryPointRotated = geometryPoint.rotateBy(rotation);
        lowestPoint = Math.min(lowestPoint, geometryPointRotated.getY() - deckHeight);
      }
    }

    //  lowestPoint is the depth below the centre of rotation
    // -lowestPoint is therefore the height above the ground
    return -lowestPoint;
  }

  /**
   * Sets the Diffector arm to rotate the safest path to the target angle, with protection against over-rotation. 
   * Below a threshold will go shortest path, otherwise will minimise total rotations
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public double goToAngle(double newAngle, double currentAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(currentAngle, 360), -180, 180);

    if (Math.abs(offset) >= turnBackThreshold)
    {
      reverseOffset = offset - Math.copySign(360, offset);

      if (Math.abs(currentAngle + offset) > Math.abs(currentAngle + reverseOffset))
        {return (currentAngle + reverseOffset);}
      
      else 
        {return (currentAngle + offset);}
    }
    else if (currentAngle + offset > maxAbsPos)
      {return (currentAngle + offset - 360);}

    else if (currentAngle + offset < -maxAbsPos)
      {return (currentAngle + offset + 360);}

    else
      {return (currentAngle + offset);}
  }

  /**
   * Sets the Diffector arm to rotate the shortest path to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public double goShortest(double newAngle, double currentAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(currentAngle, 360), -180, 180);

    if (currentAngle + offset > maxAbsPos)
      {return (currentAngle + offset - 360);}

    else if (currentAngle + offset < -maxAbsPos)
      {return (currentAngle + offset + 360);}

    else
      {return (currentAngle + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Clockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public double goClockwise(double newAngle, double currentAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(currentAngle, 360), -360, 0);

    if (currentAngle + offset > maxAbsPos)
      {return (currentAngle + offset - 360);}

    else if (currentAngle + offset < -maxAbsPos)
      {return (currentAngle + offset + 360);}

    else
      {return (currentAngle + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Anticlockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   * @param currentAngle Current/starting angle of the arm, degrees anticlockwise
   */
  public double goAnticlockwise(double newAngle, double currentAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(currentAngle, 360), 0, 360);

    if (currentAngle + offset > maxAbsPos)
      {return (currentAngle + offset - 360);}

    else if (currentAngle + offset < -maxAbsPos)
      {return (currentAngle + offset + 360);}

    else
      {return (currentAngle + offset);}
  }

}
