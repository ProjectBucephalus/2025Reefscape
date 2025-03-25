package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/** 
 * Represents a 2D position with elevation (Z) and angle (R).
 * <p>Positive Z is assumed to be up, positive R is assumed to be degrees Anticlockwise from upright. 
 * <p>For inversions, the position is assumed to be for the Port side of the robot by default.
 */
public class ArmPos 
{
  private final double Z;
  private final double R;
  private final boolean portside;

  /**
   * Constructs an ArmPos object with the given values
   * @param Z Elevation, metres
   * @param R Angle, degrees anticlockwise
   * @param portside True if the position is on the Port side of the robot
   */
  public ArmPos(double Z, double R, boolean portside)
  {
    this.Z = Z;
    this.R = R;
    
    this.portside = portside;
  }

  /** Constructs an ArmPos object with 0 values */
  public ArmPos()
  {
    this(0, 0, true);
  }

  /**
   * Constructs an ArmPos object with the given values
   * @param Z Elevation, metres
   * @param R Angle, degrees anticlockwise
   */
  public ArmPos(double Z, double R)
  {
    this(Z, R, true);
  }


  /**
   * Constructs an ArmPos object from the provided Translation2d
   * @param translation Assumes X/Y are directly mapped to Z/R
   */
  public ArmPos(Translation2d translation)
  {
    this(translation.getX(), translation.getY(), true);
  }

  /**
   * Constructs an ArmPos object from the provided Translation2d
   * @param translation Assumes X/Y are directly mapped to Z/R
   * @param portside True if the position is on the Port side of the robot
   */
  public ArmPos(Translation2d translation, boolean portside)
  {
    this(translation.getX(), translation.getY(), portside);
  }

  /** Returns the Z elevation component of the position */
  public double getZ()
    {return Z;}

  /** Returns the R angle component of the position */
  public double getR()
    {return R;}

  /** Returns the Port equivalent of the position */
  public ArmPos port()
  {
    if (portside)
      {return this;}

    return new ArmPos(Z, flip(), false);
  }

  /** Returns the Starboard equivalent of the position */
  public ArmPos stbd()
  {
    if (portside)
      {return new ArmPos(Z, flip(), false);}

    return this;
  }
  
  /** Returns the angle component, inverted across 0, wrapped [0..360] */
  public double flip()
    {return flip(R);}

  /** Returns the input angle inverted across 0, wrapped [0..360] */
  public static double flip(double angle)
    {return (360 - wrap(angle));}

  /** Wraps the input angle [0..360] */
  public static double wrap(double angle)
    {return Conversions.mod(angle, 360);}

  /**
   * Checks equality between this ArmPos and another ArmPos or Translation2d object.
   *
   * @param other The other object.
   */
  public boolean equals(ArmPos other)
    {return (other.getZ() == Z && other.getR() == R);}
  
  /**
   * Checks equality between this ArmPos and another ArmPos or Translation2d object.
   *
   * @param other The other object.
   */
  public boolean equals(Translation2d other)
    {return (other.getX() == Z && other.getY() == R);}

  /**
   * Checks equality between this ArmPos and another ArmPos based on the relative angle and inversions.
   *
   * @param other The other object.
   * @return true if the two positions are at the same elevation and relative angle, or are mirrors of eachother
   */
  public boolean relativeEquals(ArmPos other)
  {
    return
    (
      equals(other) ||
      (
        other.getZ() == Z &&
        (
          wrap(other.getR()) == wrap(R) ||
          wrap(other.getR()) == flip()
        )
      )
    );
  }
}
