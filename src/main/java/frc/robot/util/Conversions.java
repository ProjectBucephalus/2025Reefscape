package frc.robot.util;

import edu.wpi.first.math.MathUtil;

/**
 * Wraps int value over [min..max]
 */
public class Conversions 
{
    public static int wrap(int value, int min, int max)
    {
        if(value < min)
        {
            value += ((max-min) + 1);
            value = wrap(value, min, max);
        }
        else if(value > max)
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
    public static double RPSToMPS(double wheelRPS, double circumference){
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMPS Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference){
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference){
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference){
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }
}