package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;


public class DiscoLayer 
{
  private double start;
  private double length;
  public int age;
  public Color shade;
  private double velocity;
  private double accel;
  private double growth;
  private double growthRate;
  private int maxPos;
  private int maxLen;
  private double maxVel;
  private double maxGrow;
  private double maxAcc;
  private double maxGrowRate;

  public int getStartLED()
  {
    return (int)Math.floor(start);
  }

  public int getLength()
  {
    return (int)Math.floor(length);
  }

  public Color getShade()
  {
    return shade;
  }

  public int getAge()
  {
    return age;
  }

  public void update()
  {
    start += velocity;
    velocity += accel;
    length += growth;
    growth += growthRate;
    age++;
    if (start > maxPos) {start -= (maxPos + 1);}
    if (start < 0) {start += maxPos + 1;}
    velocity = Conversions.clamp(velocity, -maxVel, maxVel);
    if ((velocity == -maxVel) || (velocity == maxVel)) {accel = -accel;}
    length = Conversions.clamp(length, 1, maxLen);
    growth = Conversions.clamp(growth, -maxGrow, maxGrow);
    if ((length == 1) || (length == maxLen)) {growth = -growth;}
    else if ((growth == -maxGrow) || (growth == maxGrow)) {growthRate = -growthRate;}
    if (Math.random() > 0.9)
    {
      accel += (Math.random() - 0.5) * 0.01;
      accel = Conversions.clamp(accel, -maxAcc, maxAcc);
    }
    if (Math.random() > 0.9)
    {
      growthRate += (Math.random() -0.5) * 0.01;
      growthRate = Conversions.clamp(growthRate, -maxGrowRate, maxGrowRate);
    }
  }

  public DiscoLayer(int viewWidth)
  {
    start = Math.random()*viewWidth;
    length = Math.random()*(viewWidth-1);
    age = 0;
    do
    {
      shade = new Color(Math.random(), Math.random(), Math.random());
    } while ((shade.blue + shade.green + shade.red)<1.0);
    velocity = Math.random() - 0.5;
    accel = (Math.random() - 0.5) * 0.2;
    growth = Math.random() - 0.5;
    growthRate = (Math.random() - 0.5) * 0.2;
    maxPos = viewWidth - 1;
    maxLen = (int)Math.floor((double)viewWidth * 0.9);
    maxVel = (double)viewWidth * 0.03;
    maxAcc = 0.1;
    maxGrow = (double)viewWidth * 0.02;
    maxGrowRate = 0.05;
  }

}
