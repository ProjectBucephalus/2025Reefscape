// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CTREConfigs;
import frc.robot.constants.Constants;

public class Diffector extends SubsystemBase 
{
  public enum CargoStates{empty, oneItem, twoItem}
  /* Name is effect of motor when running clockwise/positive (e.g. elevator Up, arm Clockwise) */
  /** starboardside motor(?), forward direction drives carriage up and clockwise */
  private static TalonFX m_diffectorUC;
  /** portside motor(?), forward direction drives carriage down and clockwise */
  private static TalonFX m_diffectorDC;
  private final MotionMagicVoltage motionMagicRequester;
  private double targetElevation;
  private double targetAngle;
  private static double rotationRatio;
  private static double travelRatio;
  private double[] motorTargets = new double[2];
  private TalonFXConfiguration motorConfig;
  private CargoStates cargoState;
  private boolean coral;
  private boolean algae;
  private double maxAbsPos = Constants.Diffector.maxAbsPos;
  private double turnBackThreshold = Constants.Diffector.turnBackThreshold;
  private double stowThreshold = Constants.Diffector.angleTolerance;
  private double offset;
  private double altOffset;
  private double armPos;
  private double elevatorPos;
  
  
  /** Creates a new Diffector. */
  public Diffector() 
  {
    armConstructor();
    motorConfig = CTREConfigs.diffectorFXConfig;

    m_diffectorUC = new TalonFX(Constants.Diffector.ucMotorID);
    m_diffectorDC = new TalonFX(Constants.Diffector.uaMotorID);

    targetElevation = Constants.Diffector.startingElevation;
    targetAngle = 0;

    m_diffectorUC.getConfigurator().apply(motorConfig);
    m_diffectorDC.getConfigurator().apply(motorConfig);

    rotationRatio = Constants.Diffector.rotationRatio;
    travelRatio = Constants.Diffector.travelRatio;

    cargoState = Constants.Diffector.startingCargoState;

    motionMagicRequester = new MotionMagicVoltage(0);
  }

  /**
   * Calculates arm rotation based on motor positions
   * @return Arm rotation, degrees clockwise, 0 = algae at top
   */
  public static double getArmPos()
  {
    return (m_diffectorUC.getPosition().getValueAsDouble() + m_diffectorDC.getPosition().getValueAsDouble()) * rotationRatio;
  }

  /**
   * Calculates elevator height based on motor positions
   * @return Elevator height in m
   */
  public static double getElevatorPos()
  {
    return ((m_diffectorUC.getPosition().getValueAsDouble() - m_diffectorDC.getPosition().getValueAsDouble()) * travelRatio);
  }

  /**
   * Calculates the position to drive each motor to, based on the target positions for the elevator and arm
   * @param elevatorTarget
   * @param armTarget
   * @return
   */
  public double[] calculateMotorTargets(double elevatorTarget, double armTarget)
  {
    // IK projection and object avoidance
    double[] projectedTargets = pathfindIK(elevatorTarget, armTarget, elevatorPos, armPos);

    double[] calculatedTargets = new double[2];

    calculatedTargets[0] = (projectedTargets[1] / rotationRatio) + (projectedTargets[0] / travelRatio);
    calculatedTargets[1] = (projectedTargets[1] / rotationRatio) - (projectedTargets[0] / travelRatio);

    return calculatedTargets;
  }

  private void calculateMotorTargets()
  {
    motorTargets = calculateMotorTargets(targetElevation, targetAngle);
  }

  /**
   * Calculates Inverse-Kinematics of the diffector arm to project the best path from current to target positions
   * @param elevationTarget target height of elevator carriage, metres above deck
   * @param angleTarget target angle of the arm, degrees anticlockwise, 0 = unwound with coral at top
   * @param elevationCurrent current height of the elevator carriage, metres above deck
   * @param angleCurrent current angle of the arm, degrees
   * @return double array containing the projected path, elevation/angle pairs
   */
  private double[] pathfindIK(double elevationTarget, double angleTarget, double elevationCurrent, double angleCurrent)
  {
    

    return new Double[2] = {elevationTarget, angleTarget};
  }

  private Translation2d railPortMedial;
  private Translation2d railPortLateral;
  private Translation2d railStbdMedial;
  private Translation2d railStbdLateral;
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
  private Translation2d algaeIntersectA, algaeIntersectC;
  private Translation2d algaeWheelA, algaeWheelC;

  private Translation2d[] armGeometry, armGeometryRotated;

  private void armConstructor()
  {
    railHeight    = 0.23;
    railLateral   = 0.38;
    railMedial    = 0.27;
    deckHeight    = 0.18;
    harpoonHeight = 0.1;
    harpoonAngle  = 17;

    railStbdMedial  = new Translation2d(railMedial,railHeight);
    railStbdLateral = new Translation2d(railLateral,railHeight);
    railPortMedial  = new Translation2d(-railMedial,railHeight);
    railPortLateral = new Translation2d(-railLateral,railHeight);

    coralArmLength   = 0.5;
    coralArmAngle    = 64/2;
    algaeArmLength   = 0.6;
    algaeArmAngle    = 60/2;
    algaeWheelLength = 0.54;
    algaeClawLength  = 0.48;
    algaeInnerLength = 0.3;
    algaeInnerAngle  = 108/2;

    coralA      = new Translation2d(0,coralArmLength)  .rotateBy(Units.radiansFromDegrees(coralArmAngle));
    coralC      = new Translation2d(0,coralArmLength)  .rotateBy(Units.radiansFromDegrees(-coralArmAngle));
    algaeOuterA = new Translation2d(0,algaeArmLength)  .rotateBy(Units.radiansFromDegrees(180+algaeArmAngle));
    algaeOuterC = new Translation2d(0,algaeArmLength)  .rotateBy(Units.radiansFromDegrees(180-algaeArmAngle));
    algaeInnerA = new Translation2d(0,algaeInnerLength).rotateBy(Units.radiansFromDegrees(180+algaeInnerAngle));
    algaeInnerC = new Translation2d(0,algaeInnerLength).rotateBy(Units.radiansFromDegrees(180-algaeInnerAngle));
    algaeClawA  = new Translation2d(0,algaeClawLength) .rotateBy(Units.radiansFromDegrees(180+algaeArmAngle));
    algaeClawC  = new Translation2d(0,algaeClawLength) .rotateBy(Units.radiansFromDegrees(180-algaeArmAngle));
    algaeWheelA = new Translation2d(0,algaeWheelLength).rotateBy(Units.radiansFromDegrees(180+algaeArmAngle));
    algaeWheelC = new Translation2d(0,algaeWheelLength).rotateBy(Units.radiansFromDegrees(180-algaeArmAngle));

    armGeometry =
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
    }

    armGeometryRotated = armGeometry;
  }

  /**
   * Returns the minimum safe arm height for a given angle
   */
  private double checkPosition(double rotation)
  {
    rotation %= 360;
    for(int i = 0; i < armGeometry.size; i++)
    {
      armGeometryRotated[i] = armGeometry[i].rotateBy(Units.radiansFromDegrees(rotation));
    }
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
            Math.sqrt(Math.pow(coralArmLength,2) - Math.pow(railStbdMedial,2)) + railHeight 
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
            Math.sqrt(Math.pow(coralArmLength,2) - Math.pow(railStbdMedial,2)) + railHeight 
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

  public boolean armAtAngle()
  {
    return Math.abs(getArmPos() - targetAngle) < Constants.Diffector.angleTolerance;
  }

  public boolean elevatorAtElevation()
  {
    return Math.abs(getElevatorPos() - targetElevation) < Constants.Diffector.elevationTolerance;
  }

  /** 
   * Sets the Diffector arm to unwind to starting position 
   * @return Safe to stow
   */
  public boolean unwind()
  {
      targetAngle = Constants.Diffector.returnPos;
      return (Math.abs(armPos) < stowThreshold);
  }

  /**
   * Sets the Diffector arm to rotate the shortest path to the target angle, with protection against over-rotation
   * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goShortest(double targetAngle)
  {
      targetAngle %= 360;
      offset = MathUtil.inputModulus(targetAngle - (armPos % 360), -180, 180);

      if (armPos + offset > maxAbsPos)
          {targetAngle = (armPos + offset - 360);}

      else if (armPos + offset < -maxAbsPos)
          {targetAngle = (armPos + offset + 360);}

      else
          {targetAngle = (armPos + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Clockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goClockwise(double targetAngle)
  {
      targetAngle %= 360;
      offset = MathUtil.inputModulus(targetAngle - (armPos % 360), -360, 0);

      if (armPos + offset > maxAbsPos)
          {targetAngle = (armPos + offset - 360);}

      else if (armPos + offset < -maxAbsPos)
          {targetAngle = (armPos + offset + 360);}

      else
          {targetAngle = (armPos + offset);}
  }

  /**
   * Sets the Diffector arm to rotate Anticlockwise (viewed from bow) to the target angle, with protection against over-rotation
   * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
   */
  public void goAntiClockwise(double targetAngle)
  {
      targetAngle %= 360;
      offset = MathUtil.inputModulus(targetAngle - (armPos % 360), 0, 360);

      if (armPos + offset > maxAbsPos)
          {targetAngle = (armPos + offset - 360);}

      else if (armPos + offset < -maxAbsPos)
          {targetAngle = (armPos + offset + 360);}

      else
          {targetAngle = (armPos + offset);}
  }

    /**
     * Sets the Diffector arm to rotate the safest path to the target angle, with protection against over-rotation. 
     * Below a threshold will go shortest path, otherwise will minimise total rotations
     * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
     */
    public void goToAngle(double targetAngle)
    {
        targetAngle %= 360;
        offset = MathUtil.inputModulus(targetAngle - (armPos % 360), -180, 180);

      if (Math.abs(offset) >= turnBackThreshold)
      {
          altOffset = offset - Math.copySign(360, offset);

          if (Math.abs(armPos + offset) > Math.abs(armPos + altOffset))
              {targetAngle = (armPos + altOffset);}
          
          else 
              {targetAngle = (armPos + offset);}
      }
      else if (armPos + offset > maxAbsPos)
          {targetAngle = (armPos + offset - 360);}

      else if (armPos + offset < -maxAbsPos)
          {targetAngle = (armPos + offset + 360);}

      else
          {targetAngle = (armPos + offset);}
  }

  public double getArmTarget()
  {
    return targetAngle;
  }

  public void setElevatorTarget(double newTarget)
  {
    targetElevation = newTarget;
  }

  public double getElevatorTarget()
  {
    return targetElevation;
  }

  public void setCargoState(CargoStates newCargoState)
  {
    cargoState = newCargoState;
  }

  public CargoStates getCargoState()
  {
    return cargoState;
  }

  public void setCoral(boolean coralState)
  {
    coral = coralState;
    updateCargoState();
  }

  public void setAlgae(boolean algaeState)
  {
    algae = algaeState;
    updateCargoState();
  }

  private void updateCargoState()
  {
    if(coral && algae)
    {
      setCargoState(CargoStates.twoItem);
    }
    else if(coral || algae)
    {
      setCargoState(CargoStates.oneItem);
    }
    else if(!coral && !algae)
    {
      setCargoState(CargoStates.empty);
    }
  }

  private int getSlot()
  {
    switch (cargoState) 
    {
      case empty: return 0;
      case oneItem: return 1;
      case twoItem: return 2;
      default: return 0;
    }
  }

  @Override
  public void periodic() 
  { 
    calculateMotorTargets();

    m_diffectorUC.setControl(motionMagicRequester.withPosition(motorTargets[0]).withSlot(getSlot()));
    m_diffectorDC.setControl(motionMagicRequester.withPosition(motorTargets[1]).withSlot(getSlot()));

    armPos = getArmPos();
    elevatorPos = getElevatorPos()

    SmartDashboard.putNumber("Elevator Height", getElevatorPos());
    SmartDashboard.putNumber("Arm Rotation", getArmPos());
  }
}
