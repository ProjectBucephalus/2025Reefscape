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
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DiffectorConstants.IKGeometry;
import frc.robot.subsystems.Diffector;

/** Add your docs here. */
public class ArmCalculator 
{
  private double minElevation;
  private double maxElevation;
  private double safeElevation;
  private double projectionAngle;
  private double projectionElevation;
  private double waypointHold;

  private double railHeight;
  private double railLateral;
  private double railMedial;
  private double deckHeight;
  private double harpoonHeight;
  private double harpoonAngle;
  
  private double coralArmLength;
  private double coralArmAngle;
  private double algaeArmLength;
  private double algaeArmAngle;
  private double algaeWheelLength;
  private double algaeClawLength;
  private double algaeInnerLength;
  private double algaeInnerAngle;
  private double offset;
  private double angle; 
  private double maxAbsPos;
  private double reverseOffset;
  private double turnBackThreshold;

  private Translation2d coralA, coralC;
  private Translation2d algaeOuterA, algaeOuterC;
  private Translation2d algaeInnerA, algaeInnerC;
  private Translation2d algaeClawA, algaeClawC;
  private Translation2d algaeWheelA, algaeWheelC;

  /** Unrotated virtual arm */
  private final Translation2d[] armGeometry;
  /** Rotatable virtual arm */
  private Translation2d[] armGeometryRotated;

  public ArmCalculator()
  {
    maxElevation  = Constants.DiffectorConstants.maxElevation;
    minElevation  = Constants.DiffectorConstants.minElevation;
    projectionAngle = IKGeometry.projectionAngle;
    projectionElevation = IKGeometry.projectionElevation;
    
    railHeight    = IKGeometry.railHeight;
    railLateral   = IKGeometry.railLateral;
    railMedial    = IKGeometry.railMedial;
    deckHeight    = IKGeometry.deckHeight;
    harpoonHeight = IKGeometry.harpoonHeight;
    harpoonAngle  = IKGeometry.harpoonAngle;
    
    coralArmLength   = IKGeometry.coralArmLength;
    coralArmAngle    = IKGeometry.coralArmAngle;
    algaeArmLength   = IKGeometry.algaeArmLength;
    algaeArmAngle    = IKGeometry.algaeArmAngle;
    algaeWheelLength = IKGeometry.algaeWheelLength;
    algaeClawLength  = IKGeometry.algaeClawLength;
    algaeInnerLength = IKGeometry.algaeInnerLength;
    algaeInnerAngle  = IKGeometry.algaeInnerAngle;

    safeElevation = deckHeight + Math.max(coralArmLength, algaeArmLength);
    
    coralA      = new Translation2d(0,coralArmLength)  .rotateBy(new Rotation2d(Units.degreesToRadians(coralArmAngle)));
    coralC      = new Translation2d(0,coralArmLength)  .rotateBy(new Rotation2d(Units.degreesToRadians(-coralArmAngle)));
    algaeOuterA = new Translation2d(0,algaeArmLength)  .rotateBy(new Rotation2d(Units.degreesToRadians(180+algaeArmAngle)));
    algaeOuterC = new Translation2d(0,algaeArmLength)  .rotateBy(new Rotation2d(Units.degreesToRadians(180-algaeArmAngle)));
    algaeInnerA = new Translation2d(0,algaeInnerLength).rotateBy(new Rotation2d(Units.degreesToRadians(180+algaeInnerAngle)));
    algaeInnerC = new Translation2d(0,algaeInnerLength).rotateBy(new Rotation2d(Units.degreesToRadians(180-algaeInnerAngle)));
    algaeClawA  = new Translation2d(0,algaeClawLength) .rotateBy(new Rotation2d(Units.degreesToRadians(180+algaeArmAngle)));
    algaeClawC  = new Translation2d(0,algaeClawLength) .rotateBy(new Rotation2d(Units.degreesToRadians(180-algaeArmAngle)));
    algaeWheelA = new Translation2d(0,algaeWheelLength).rotateBy(new Rotation2d(Units.degreesToRadians(180+algaeArmAngle)));
    algaeWheelC = new Translation2d(0,algaeWheelLength).rotateBy(new Rotation2d(Units.degreesToRadians(180-algaeArmAngle)));
    
    armGeometry = new Translation2d[]
    {
      coralA,      // [0] // Anticlockwise limit of Coral arm
      coralC,      // [1] // Clockwise limit of Coral arm
      algaeOuterA, // [2] // Anticlockwise limit of Algae arm
      algaeOuterC, // [3] // Clockwise limit of Algae arm
      algaeInnerA, // [4] // Anticlockwise, innermost point of Algae claw
      algaeInnerC, // [5] // Clockwise, innermost point of Algae claw
      algaeClawA,  // [6] // Anticlockwise, outermost point of Algae claw
      algaeClawC,  // [7] // Clockwise, outermost point of Algae claw
      algaeWheelA, // [8] // Anticlockwise limit of Algae wheel
      algaeWheelC  // [9] // Clockwise limit of Algae wheel
    };
    
    armGeometryRotated = armGeometry.clone();
  }

  /**
   * Calculates Inverse-Kinematics of the diffector arm to project the best path from current to target positions
   * @param elevationTarget target height of elevator carriage, metres above deck
   * @param angleTarget target angle of the arm, degrees anticlockwise, 0 = unwound with coral at top
   * @param elevationCurrent current height of the elevator carriage, metres above deck
   * @param angleCurrent current angle of the arm, degrees anticlockwise, 0 = unwound with coral at top
   * @return double array containing the projected path, elevation/angle pairs
   */
  public ArrayList<Translation2d> pathfindArm(double elevationTarget, double angleTarget, double elevationCurrent, double angleCurrent)
  {
    elevationCurrent = Conversions.clamp(elevationCurrent,minElevation,maxElevation);
    elevationTarget  = Conversions.clamp(elevationTarget,minElevation,maxElevation);

    ArrayList<Translation2d> pathOutput = new ArrayList<Translation2d>();

    // Full intended path of arm is above safe limits, path is safe as given
    if (elevationCurrent >= safeElevation && elevationTarget >= safeElevation)
    {
      pathOutput.add(new Translation2d(elevationTarget, angleTarget));
      return pathOutput;
    }

    double angleChange = angleTarget - angleCurrent;
    double angleRelative = Conversions.mod(angleCurrent, 360);

    ArrayList<Double> waypointList = new ArrayList<Double>();

    // Elevation change only
    if (Math.abs(angleChange) <= Constants.DiffectorConstants.angleTolerance)
    {
      pathOutput.add(new Translation2d(checkPosition(elevationTarget, angleTarget), angleTarget));
      return pathOutput;
    }

    // Arm is not vertical:
    else if
    (
      Conversions.mod(angleRelative, 180) > Constants.DiffectorConstants.angleTolerance && 
      Conversions.mod(angleRelative, 180) < 180 - Constants.DiffectorConstants.angleTolerance  
    )
    {
      // Anticlockwise rotation past both verticals:
      if 
      (
        // Angle change greater than one rotation
        angleChange >= 360 ||
        // Or relative angle change goes past both verticals
        (angleRelative < 180 && angleRelative + angleChange >= 360)
      )
      {
        // Intermediate waypoint: Safe elevation, rotated to the last vertical point before the target
        waypointList.add(safeElevation);
        waypointList.add(angleTarget - Conversions.mod(angleTarget, 180));
      }

      // Clockwise rotation past both verticals:
      else if 
      (
        // Angle change greater than one rotation
        angleChange <= -360 ||
        // Or relative angle change goes past both verticals
        (angleRelative > 180 && angleRelative + angleChange <= 0)
      )
      {
        // Intermediate waypoint: Safe elevation, rotated to the last vertical point before the target
        waypointList.add(safeElevation);
        waypointList.add(angleTarget + Conversions.mod(-angleTarget, 180));
      }

      // Anticlockwise rotation taking the arm past vertical:
      else if 
      ( // Relative angle change goes past upright
        angleRelative + angleChange >= 360 || 
        // Or relative angle change goes past upside-down
        (angleRelative < 180 && angleRelative + angleChange >= 180)
      )
      {
        // Intermediate waypoint: Safe elevation for the last vertical point before the target
        waypointList.add(checkPosition(elevationTarget, angleTarget - Conversions.mod(angleTarget, 180)));
        waypointList.add(angleTarget - Conversions.mod(angleTarget, 180));
      }
      
      // Clockwise rotation taking the arm past vertical:
      else if 
      ( // Relative angle change goes past upright
        angleRelative + angleChange <= 0 || 
        // Or relative angle change goes past upside-down
        (angleRelative > 180 && angleRelative + angleChange <= 180)
      )
      {
        // Intermediate waypoint: Safe elevation for the last vertical point before the target
        waypointList.add(checkPosition(elevationTarget, angleTarget + Conversions.mod(-angleTarget, 180)));
        waypointList.add(angleTarget + Conversions.mod(-angleTarget, 180));
      }

      // else: Angle change does not take arm past vertical
      //       therefore the safe height is greatest at one end
      //       and no intermediate waypoint is needed
    }
    // Unless stowed, ensure the arm is safe before moving from vertical
    else if (!Diffector.transferRequested && elevationCurrent <= checkAngle(angleCurrent))
    {
      waypointList.add(checkAngle(angleCurrent) + projectionAngle);
      waypointList.add(angleCurrent);
    }

    // Add Target waypoint:
    waypointList.add(checkPosition(elevationTarget, angleTarget));
    waypointList.add(angleTarget);

    // Immediate path projection to protect against dangerous elevation/rotation sequencing
    // Immediate rotation would cause collision:
    if (checkAngle(angleCurrent + Math.copySign(Math.min(projectionAngle, Math.abs(angleChange)), angleChange)) > elevationCurrent)
    { // Set initial waypoint halfway between initial and first waypoint elevations without rotating
      waypointHold = waypointList.get(0);
      waypointList.add(0, Math.max(Math.max((elevationCurrent + waypointHold) / 2, elevationCurrent), waypointHold));
      waypointList.add(1, angleCurrent);
    }
    // Immediate drop in elevation would cause collision:
    else if (waypointList.get(0) < elevationCurrent && checkAngle(angleCurrent) > elevationCurrent - projectionElevation)
    { // Set initial waypoint halfway between initial and first waypoint rotations without elevating
      waypointHold = waypointList.get(1);
      waypointList.add(0, elevationCurrent);
      waypointList.add(1, (angleCurrent + waypointHold) / 2);
    }

    // Convert waypoint ArrayList to Translation2d array to return
    for (int i = 0; i < waypointList.size() / 2; i++) 
      {pathOutput.add(new Translation2d(waypointList.get(2 * i), waypointList.get((2 * i) + 1)));}
    return pathOutput;
  }

  /**
   * Adjusts the elevation of a given position to keep above safe limits
   * @param elevation the intended elevation
   * @param angle the angle of the arm to check
   * @return maximum of the intended elevation and the safe elevation for the given angle
   */
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

    // Starting stow position puts the Algae arm below deck
    if (angle == Constants.DiffectorConstants.startAngle && !RobotContainer.algae)
      {return Constants.DiffectorConstants.startElevation;}

    angle = Conversions.mod(angle, 360);

    /*  
     *  Rotating the virtual arm reference points to allow for position calculations
     *    Some conditions are based on the angle
     *    Some on the relative height of certain reference points
     *    Some on the relative translation of certain reference points
     */
    Rotation2d rotation = new Rotation2d(Units.degreesToRadians(angle));
    for(int i = 0; i < armGeometry.length; i++)
      {armGeometryRotated[i] = armGeometry[i].rotateBy(rotation);}

    if(angle > 90 && angle < 270) // Coral arm down:
    { 
    /*
     *  Coral manipulator:
     *    36 degree arc, centred on 0
     *    0.53m radius
     */

      if (MathUtil.isNear(180, angle, harpoonAngle) && !RobotContainer.coral) // Holding coral & angle is within range of the harpoon:
        {return -Math.min(armGeometryRotated[0].getY(), armGeometryRotated[1].getY()) + deckHeight - harpoonHeight;} // Keep coral above the harpoon

      if (armGeometryRotated[0].getX() >= 0 && armGeometryRotated[1].getX() <= 0) // Coral arm extends to either side of the mast:
        {return -Math.min(armGeometryRotated[0].getY(), armGeometryRotated[1].getY()) + deckHeight;} // Keep carriage above the deck by the length of the arm

      if (armGeometryRotated[0].getX() < 0) // Anticlockwise Coral limit is Starbord/Clockwise of the mast:
      {
        if (armGeometryRotated[0].getX() <= -railMedial) // Anticlockwise Coral limit is beyond the Medial rail limit:
          {return (-armGeometryRotated[0].getY()) + railHeight;} // Keep the Anticlockwise Coral limit above the rail
        else // Antilockwise Coral limit is within the Medial rail limit:
        {
          return Math.max
          (
            // Keep the Anticlockwise Coral limit above the deck
            (-armGeometryRotated[0].getY()) + deckHeight,
            // and keep the carriage away from the rail by the length of the arm
            Math.sqrt(Math.pow(coralArmLength,2) - Math.pow(railMedial,2)) + railHeight 
          );
        }
      }
      else // Clockwise Coral limit is Port/Anticlockwise of the mast:
      {
        if (armGeometryRotated[1].getX() >= railMedial) // Clockwise Coral limit is beyond the Medial rail limit:
          {return (-armGeometryRotated[1].getY()) + railHeight;} // Keep the Clockwise Coral limit above the rail
        else // Clockwise Coral limit is within the Medial rail limit:
        {
          return Math.max
          (
            // Keep the Clockwise Coral limit above the deck
            (-armGeometryRotated[1].getY()) + deckHeight,
            // and keep the carriage away from the rail by the length of the arm
            Math.sqrt(Math.pow(coralArmLength,2) - Math.pow(railMedial,2)) + railHeight 
          );
        }
      }
    }

    else if (angle < 90 || angle > 270) // Algae arm down:
    {
    /*
     *  Algae manipulator:
     *    Primary span:
     *      60 degree arc, centred on 180
     *      0.6m radius
     *    Claw:
     *      Inner span:
     *        108 degree arc, centred on 180
     *        0.3m radius, then projected paralel to arm
     *      Outerpoint of wheels halfway between the limits of the primary span, and the point where the Claw meets the primary span
     */
      
      if (armGeometryRotated[2].getX() >= -0 && armGeometryRotated[3].getX() <= 0) // Algae arm extends to either side of the mast:
        {return algaeArmLength + deckHeight;} // Keep carriage above the deck by the length of the arm
      
      if (armGeometryRotated[2].getX() < 0) // Anticlockwise Algae limit is Starbord/Clockwise of the mast:
      {
        if (armGeometryRotated[2].getX() >= -railMedial) // Anticlockwise Algae limit is within the Starboard rail:
        {
          {
            return Math.max
            (
              // Keep the Anticlockwise Algae limit above the deck
              (-armGeometryRotated[2].getY()) + deckHeight,
              // and keep the carriage away from the rail by the length of the arm
              Math.sqrt(Math.pow(algaeArmLength,2) - Math.pow(railMedial,2)) + railHeight
            );
          }
        }
        else if (armGeometryRotated[6].getX() <= -railLateral) // The Anticlockwise wheel is beyond the rail
        {
          return 
            Math.max(-armGeometryRotated[6].getY(), -armGeometryRotated[4].getY()) 
            + railHeight; // Keep inner and outer ends of the claw above the rail
        }
        else if (armGeometryRotated[2].getX() <= -railLateral) // Anticlockwise limit is beyond the rail:
          {return -armGeometryRotated[8].getY() + railHeight;} // Keep the Anticlockwise wheel above the rail
        else                                                   // Anticlockwise limit is within the rail:
          {return -armGeometryRotated[2].getY() + railHeight;} // Keep the Anticlockwise limit above the rail
      }
      
      else // Clockwise Algae limit is Port/Anticlockwise of the mast:
      {
        if (armGeometryRotated[3].getX() <= railMedial) // Clockwise Algae limit is within the rail:
        {
          {
            return Math.max
            (
              // Keep the Clockwise Algae limit above the deck
              (-armGeometryRotated[3].getY()) + deckHeight,
              // and keep the carriage away from the rail by the length of the arm
              Math.sqrt(Math.pow(algaeArmLength,2) - Math.pow(railMedial,2)) + railHeight
            );
          }
        }
        else if (armGeometryRotated[7].getX() >= railLateral) // The Clockwise wheel is beyond the rail:
        {
          return 
            Math.max(-armGeometryRotated[7].getY(), -armGeometryRotated[5].getY()) 
            + railHeight; // Keep inner and outer ends of the claw above the rail
        }
        else if (armGeometryRotated[3].getX() >= railLateral)  // Clockwise limit is beyond the rail:
          {return -armGeometryRotated[9].getY() + railHeight;} // Keep the Clockwise wheel above the rail
        else                                                   // Clockwise limit is within the rail:
          {return -armGeometryRotated[3].getY() + railHeight;} // Keep the Clockwise limit above the rail
      }
    }

    else // Arm is horizontal
      {return minElevation;}
  }

  public Translation2d pathFollow(ArrayList<Translation2d> pathPoints, Translation2d currentPoint)
  {
    if (pathPoints.size() == 0)
      {return currentPoint;}
    if (pathPoints.size() == 1)
      {return pathPoints.get(0);}
    int pathIndex = 0;
    while (pathIndex < pathPoints.size() - 1)
    {
      if (pathPoints.get(pathIndex).getDistance(currentPoint) > pathPoints.get(pathIndex + 1).getDistance(currentPoint))
        {pathIndex++;}
      else
        {break;}
    }


    return pathPoints.get(pathIndex);
  }

  /**
   * Sets the Diffector arm to rotate the safest path to the target angle, with protection against over-rotation. 
   * Below a threshold will go shortest path, otherwise will minimise total rotations
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public double goToAngle(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(angle, 360), -180, 180);

    if (Math.abs(offset) >= turnBackThreshold)
    {
      reverseOffset = offset - Math.copySign(360, offset);

      if (Math.abs(angle + offset) > Math.abs(angle + reverseOffset))
        {return (angle + reverseOffset);}
      
      else 
        {return (angle + offset);}
    }
    else if (angle + offset > maxAbsPos)
      {return (angle + offset - 360);}

    else if (angle + offset < -maxAbsPos)
      {return (angle + offset + 360);}

    else
      {return (angle + offset);}
  }

  /**
   * Sets the Diffector arm to rotate the shortest path to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public double goShortest(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(angle, 360), -180, 180);

    if (angle + offset > maxAbsPos)
      {return (angle + offset - 360);}

    else if (angle + offset < -maxAbsPos)
      {return (angle + offset + 360);}

    else
      {return (angle + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Clockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public double goClockwise(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(angle, 360), -360, 0);

    if (angle + offset > maxAbsPos)
      {return (angle + offset - 360);}

    else if (angle + offset < -maxAbsPos)
      {return (angle + offset + 360);}

    else
      {return (angle + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Anticlockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param newAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public double goAnticlockwise(double newAngle)
  {
    newAngle = Conversions.mod(newAngle, 360);
    offset = MathUtil.inputModulus(newAngle - Conversions.mod(angle, 360), 0, 360);

    if (angle + offset > maxAbsPos)
      {return (angle + offset - 360);}

    else if (angle + offset < -maxAbsPos)
      {return (angle + offset + 360);}

    else
      {return (angle + offset);}
  }

}
