package frc.robot.util;

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
}