package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Wraps int value over [min..max]
 */
public class Conversions 
{
  /**
   * Mathematical modulus opperation, correcting the Java implimentation that incorrectly returns negative vaues
   * @param value input value
   * @param base base value of modulus
   * @return e.g. mod(8,10) == mod(18,10) == mod(-2,10) == 8
   */
  public static double mod(double value, double base)
  {
    value %= base;
    if (value < 0) {value += base;}
    return value;
  }

  public static int wrap(int value, int min, int max)
  {
    if (value < min)
    {
      value += ((max-min) + 1);
      value = wrap(value, min, max);
    }
    else if (value > max)
    {
      value -= ((max-min) + 1);
      value = wrap(value,min,max);
    }
    return  value;
  }

  /**
   * Clamps value [-1..1]
   * @param value Value to clamp
   * @return Clamped value
   */
  public static double clamp(double value)
  {
    return clamp(value, -1, 1);
  }

  /**
   * Clamps values to parameters
   * @param value Value to clamp
   * @param min Minimum value
   * @param max Maximum value
   * @return Clamped value
   */
  public static double clamp(double value, double min, double max)
  {
    return Math.min(Math.max(value, Math.min(min,max)), Math.max(min,max));
  }

  /**
   * Clamps values to parameters
   * @param value Value to clamp
   * @param min Minimum value
   * @param max Maximum value
   * @return Clamped value
   */
  public static int clamp(int value, int min, int max)
  {
    return (int) Math.min(Math.max(value, Math.min(min,max)), Math.max(min,max));
  }
  
  /**
   * @param wheelRPS Wheel Velocity: (in Rotations per Second)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Velocity: (in Meters per Second)
   */
  public static double RPSToMPS(double wheelRPS, double circumference)
  {
    double wheelMPS = wheelRPS * circumference;
    return wheelMPS;
  }

  /**
   * @param wheelMPS Wheel Velocity: (in Meters per Second)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Velocity: (in Rotations per Second)
   */
  public static double MPSToRPS(double wheelMPS, double circumference)
  {
    double wheelRPS = wheelMPS / circumference;
    return wheelRPS;
  }

  /**
   * @param wheelRotations Wheel Position: (in Rotations)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Distance: (in Meters)
   */
  public static double rotationsToMeters(double wheelRotations, double circumference)
  {
    double wheelMeters = wheelRotations * circumference;
    return wheelMeters;
  }

  /**
   * @param wheelMeters Wheel Distance: (in Meters)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Position: (in Rotations)
   */
  public static double metersToRotations(double wheelMeters, double circumference)
  {
    double wheelRotations = wheelMeters / circumference;
    return wheelRotations;
  }

  public static Translation2d flipTranslation(Translation2d position) 
  {
    // flip when red
    if (FieldUtils.isRedAlliance()) {
      // reflect the pose over center line, flip both the X
      return new Translation2d(FieldUtils.fieldLength - position.getX(), position.getY());
    }

    // Blue or we don't know; return the original position
    return position;
  }

  public static Pose2d transformToPose2d(Transform2d position) 
  {
    // Converts a Transform2d to a pose.
    return new Pose2d(position.getTranslation(), position.getRotation());
  }

  public static Pose2d translationToPose2d(Translation2d position) 
  {
    // Converts a Transform2d to a pose.
    return new Pose2d(position.getX(), position.getY(), position.getAngle());
  }
}