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
import frc.robot.constants.Constants.DiffectorConstants;
import frc.robot.constants.Constants.DiffectorConstants.IKGeometry;

/** Add your docs here. */
public class ArmCalculator 
{
  private double minElevation;
  private double maxElevation;
  private double safeElevation;
  private double projectionAngle;
  private double projectionElevation;

  private double railHeight;
  private double railLateral;
  private double railMedial;
  private double deckHeight;

  private double offset;
  private double maxAbsPos;
  private double reverseOffset;
  private double turnBackThreshold;

  /** Unrotated virtual arm */
  private final Translation2d[] armGeometry;
  
  public ArmCalculator()
  {
    maxElevation  = DiffectorConstants.maxZ;
    minElevation  = DiffectorConstants.minZ;
    safeElevation = DiffectorConstants.safeElevation;
    projectionAngle = IKGeometry.projectionAngle;
    projectionElevation = IKGeometry.projectionElevation;

    maxAbsPos = DiffectorConstants.maxAbsAngle;
    turnBackThreshold = DiffectorConstants.turnBackThreshold;
    
    railHeight    = IKGeometry.railHeight;
    railLateral   = IKGeometry.railLateral;
    railMedial    = IKGeometry.railMedial;
    deckHeight    = IKGeometry.deckHeight;

    armGeometry = IKGeometry.armGeometry;
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

    if (robotPos.getDistance(allianceReef.getCentre()) <= DiffectorConstants.IKGeometry.reefSafetyRadius) 
      {safeElevation = DiffectorConstants.reefSafeElevation;}
    else
      {safeElevation = DiffectorConstants.safeElevation;}

    if 
    ( // Certain positions put the arm lower than it would otherwise be allowed to go
      (
        relativeTarget.equals(DiffectorConstants.startPosition) ||
        relativeTarget.equals(DiffectorConstants.coralTransferPosition) ||
        relativeTarget.equals(DiffectorConstants.algaeIntakePosition) ||
        relativeTarget.equals(DiffectorConstants.processorPosition) ||
        relativeTarget.equals(DiffectorConstants.climbPosition)
      )
    )
    { // Forced safe path for unsafe targets
      pathOutput.add(new Translation2d(Math.max(safeElevation, startPosition.getX()), startPosition.getY()));
      pathOutput.add(new Translation2d(Math.max(safeElevation, startPosition.getX()), targetPosition.getY()));
      pathOutput.add(new Translation2d(safeElevation, targetPosition.getY())); // Ensuring arm is not rotating
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
    if (Math.abs(angleChange) <= DiffectorConstants.angleTolerance)
    {
      pathOutput.add(targetPosition);
      return pathOutput;
    }
    
    if // Arm is not vertical:
    (
      Conversions.mod(angleRelative, 180) > DiffectorConstants.angleTolerance && 
      Conversions.mod(angleRelative, 180) < 180 - DiffectorConstants.angleTolerance
    )
    {
      // Any rotation taking the arm past vertical:
      if 
      ( // Anticlockwise angle change goes past upright
        angleRelative + angleChange >= 360 || 
        // Anticlockwise angle change goes past upside-down
        (angleRelative < 180 && angleRelative + angleChange >= 180) ||
        // Clockwise angle change goes past upright
        angleRelative + angleChange <= 0 || 
        // Clockwise angle change goes past upside-down
        (angleRelative > 180 && angleRelative + angleChange <= 180)
      )
      {
        // Intermediate waypoint: Safe elevation at initial rotation
        pathOutput.add(new Translation2d(safeElevation, startPosition.getY()));
        pathOutput.add(new Translation2d(safeElevation, targetPosition.getY()));
      }
      
      // Rotation does not go past vertical -> never needs to go higher than start or end
      else if (startPosition.getX() < checkAngle(targetPosition.getY())) // Start is lower than is safe for final rotation
      { // Go to safe elevation for final rotation, then rotate
        pathOutput.add(new Translation2d(checkAngle(targetPosition.getY()), startPosition.getY()));
        pathOutput.add(new Translation2d(checkAngle(targetPosition.getY()), targetPosition.getY()));
      }
      else // Start is high enough to rotate to final rotation
        {pathOutput.add(new Translation2d(startPosition.getX(), targetPosition.getY()));}
    }

    // Arm starts vertical and starts lower than is safe
    else if (startPosition.getX() <= checkAngle(startPosition.getY()))
    { // Ensure the arm is safe before moving from vertical
      pathOutput.add(new Translation2d(safeElevation, startPosition.getY()));
      pathOutput.add(new Translation2d(safeElevation, targetPosition.getY()));
    }

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
    {return Conversions.clamp(position.getX(), checkAngle(position.getY()), maxElevation);}

  public double checkPosition(double elevation, double angle)
    {return Conversions.clamp(elevation, checkAngle(angle), maxElevation);}

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
    
    for (Translation2d geometryPoint : armGeometry)
    {
      geometryPointRotated = geometryPoint.rotateBy(rotation);
      if (geometryPointRotated.getY() < 0)
      {
        if (Math.abs(geometryPointRotated.getX()) < railMedial)
          {lowestPoint = Math.min(lowestPoint, geometryPointRotated.getY() - deckHeight);} // Point is directly over the deck
        else if (Math.abs(geometryPointRotated.getX()) < railLateral)
          {lowestPoint = Math.min(lowestPoint, geometryPointRotated.getY() - railHeight);} // Point is directly over the rail
        else
          {lowestPoint = Math.min(lowestPoint, geometryPointRotated.getY() - deckHeight);} // Point is beyond the rail
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
