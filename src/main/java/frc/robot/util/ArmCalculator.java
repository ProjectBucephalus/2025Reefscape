// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants.Diffector.IKGeometry;

/** Add your docs here. */
public class ArmCalculator 
{
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

  private Translation2d coralA, coralC;
  private Translation2d algaeOuterA, algaeOuterC;
  private Translation2d algaeInnerA, algaeInnerC;
  private Translation2d algaeClawA, algaeClawC;
  private Translation2d algaeWheelA, algaeWheelC;

  private Translation2d[] armGeometry, armGeometryRotated;

  public ArmCalculator()
  {
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
      coralA,
      coralC,
      algaeOuterA,
      algaeOuterC,
      algaeInnerA,
      algaeInnerC,
      algaeClawA,
      algaeClawC,
      algaeWheelA,
      algaeWheelC
    };

    armGeometryRotated = armGeometry;
  }

  /**
   * Calculates Inverse-Kinematics of the diffector arm to project the best path from current to target positions
   * @param elevationTarget target height of elevator carriage, metres above deck
   * @param angleTarget target angle of the arm, degrees anticlockwise, 0 = unwound with coral at top
   * @param elevationCurrent current height of the elevator carriage, metres above deck
   * @param angleCurrent current angle of the arm, degrees
   * @return double array containing the projected path, elevation/angle pairs
   */
  public double[] pathfindIK(double elevationTarget, double angleTarget, double elevationCurrent, double angleCurrent)
  {
    

    return new double[] {elevationTarget, angleTarget};
  }

  /**
   * Returns the minimum safe arm height for a given angle
   */
  public double checkPosition(double angle)
  {
    angle %= 360;
    Rotation2d rotation = new Rotation2d(Units.degreesToRadians(angle));
    for(int i = 0; i < armGeometry.length; i++)
      {armGeometryRotated[i] = armGeometry[i].rotateBy(rotation);}
    
    //  Min height: 0.444 -> used for climb
    //  Max height: 1.709 -> used for net scoring
    //  Stow height: 0.574

    if(angle > 90 && angle < 270) // Coral arm down:
    { 
    //  Coral manipulator:
    //    64 degree arc, centred on 0
    //    0.5m radius

      if (armGeometryRotated[0].getX() >= 0 && armGeometryRotated[1].getX() <= 0) // Coral arm extends to either side of the mast:
        {return coralArmLength + deckHeight;} // Keep carriage above the deck by the length of the arm

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

    else // Algae arm down:
    {
    //  Algae manipulator:
    //    Primary span:
    //      60 degree arc, centred on 180
    //      0.6m radius
    //    Claw:
    //      Inner span:
    //        107 degree arc, centred on 180
    //        0.3m radius, then projected paralel to arm
    //      Outerpoint of wheels halfway between the limits of the primary span, and the point where the Claw meets the primary span
    //  Harpoon:
    //    Extends from the deck behind the plane of rotation, such that if algae is held,
    //    the minimum safe height 17 degrees either side of mast is increased by 0.1m

      if ((angle <= harpoonAngle || angle >= 360 - harpoonAngle) && RobotContainer.algae) // Holding algae & angle is within range of the harpoon:
        {return algaeArmLength + deckHeight + harpoonHeight;} // Keep algae above the harpoon
      
      if (armGeometryRotated[2].getX() >= 0 && armGeometryRotated[3].getX() <= 0) // Algae arm extends to either side of the mast:
        {return algaeArmLength + deckHeight;} // Keep carriage above the deck by the length of the arm
      
      if (armGeometryRotated[2].getX() < 0) // Anticlockwise Algae limit is Starbord/Clockwise of the mast:
      {
        if (armGeometryRotated[2].getX() >= railMedial) // Anticlockwise Algae limit is within the rail:
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
  }
}
