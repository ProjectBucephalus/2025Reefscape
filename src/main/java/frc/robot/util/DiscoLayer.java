package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.Constants.LEDStrip;


public class DiscoLayer 
{
  /**
   *  layer class object for the disco mode of lightlayer
   */
  private double start; // the floor of this value is the position at which this layer starts
  private double length; // the floor of this value is the width or length of the layer, in LED's
  public int age; // how long in seconds this layer has been "alive"
  public Color shade; // Color of the layer
  private double velocity; // speed the layer is moving, in LED's/second
  private double accel; // acceleration, velocity is updated by this each update (LED's/second/second)
  private double growth; // amount the length is changing, in LED's/second
  private double growthRate; // change in growth, growth is updated by this each update (LED's/second/second)
  private int maxPos; // maximum start position, effectively the width of the Lightlayer
  private int maxLen; // maximum length for this layer, also effectively the width of the lightlayer
  private double maxVel; // maximum absolute velocity (velocity is between -maxVel and +maxVel)
  private double maxGrow; // maximum absolute growth rate, can be +ve or -ve
  private double maxAcc; // maximum absolute acceleration
  private double maxGrowRate; // maximum absolute growth rate
  private double startTime; // system time when this layer started (used to calculate age)
  private double lastTime; // time last time .update() was called (used to calculate delta)
  private double currTime; // current system time
  private double timeDelta; // difference between current time and last time, ie: how much time has passsed since last .update()

  public int getStartLED()
  {
    /**
     * Getter for the starting LED position
     * Returns an int representing the led number this layer should start drawing at
     */
    return (int)Math.floor(start);
  }

  public int getLength()
  {
    /**
     * Getter for the length of the layer
     * Returns an int representing the number of LED's this layer should draw
     */
    return (int)Math.floor(length);
  }

  public Color getShade()
  {
    /**
     * Getter for the Color of the layer
     * Returns an WPILib.Util.Color object representing this layers colour.
     */
    return shade;
  }

  public int getAge()
  {
    /**
     * Getter for the layer age
     * Returns the time this layer has been "alive" at last update, in seconds
     */
    return age;
  }

  public void update()
  {
    /**
     * does internal maintenance and logic, should be called once each render cycle
     */
    currTime = Timer.getTimestamp();
    timeDelta = currTime - lastTime;
    start += (velocity * timeDelta);
    velocity += (accel * timeDelta);
    length += (growth * timeDelta);
    growth += (growthRate * timeDelta);
    age = (int)Math.floor(currTime - startTime);
    
    // do limit checks
    
    if (start > maxPos) {start -= (maxPos + 1);}
    if (start < 0) {start += maxPos + 1;}
    velocity = MathUtil.clamp(velocity, -maxVel, maxVel);
    
    //if we are at max velocity, flip the acceleration so it starts changing
    if (((velocity == -maxVel) && (accel < 0)) || ((velocity == maxVel) && (accel > 0))) {accel = -accel;}
    
    length = MathUtil.clamp(length, 1, maxLen);
    growth = MathUtil.clamp(growth, -maxGrow, maxGrow);
    
    // if at max or min size, flip growth so it will start changing 
    if (((length == 1) && (growth < 0)) || ((length == maxLen) && (growth > 0))) {growth = -growth;}
    
    // also if growth is at max or min, flip growth rate change
    else if (((growth == -maxGrow) && (growthRate < 0)) || ((growth == maxGrow) &&(growthRate < 0)) ) {growthRate = -growthRate;}
    
    // also throw in a random change to accel and growth rate every now and then
    if (Math.random() > LEDStrip.discoChangeChance)
    {
      accel += (Math.random() - 0.5) * LEDStrip.discoMaxAccelMultiplier; // change by up to half max accel
      accel = MathUtil.clamp(accel, -maxAcc, maxAcc); // make sure not out of bounds
    }
    if (Math.random() > LEDStrip.discoChangeChance)
    {
      growthRate += (Math.random() - 0.5) * LEDStrip.discoMaxGrowRateMultiplier; //change by up to half max growrate
      growthRate = MathUtil.clamp(growthRate, -maxGrowRate, maxGrowRate); //make sure not out of bounds
    }
  }

  public DiscoLayer(int viewWidth)
  {
    /**
     * constructor sets initial (mostly random) values.
     * viewWidth should be the width of the lightlayer being rendered to and is used to
     * calculate reasonable limits for many of the values.
     */
    start = Math.random()*viewWidth;
    length = Math.random()*(viewWidth-1);
    age = 0;
    
    // we like nice bright Colors :)
    shade = new Color(Math.random(), Math.random(), Math.random());
    
    // if all colour values are below the threshold, the colour is too dark, randomly set one of them to 1.
    if ((shade.blue < LEDStrip.discoColorThreshold) && 
        (shade.green < LEDStrip.discoColorThreshold) && 
        (shade.red < LEDStrip.discoColorThreshold))
    {
      double chance = Math.random();
      if (chance < 0.3)
      {
        shade = new Color(1.0,shade.green,shade.blue); 
      }
      else if (chance > 0.7)
      {
        shade = new Color(shade.red,shade.green,1.0);
      }
      else
      {
        shade = new Color(shade.red,1.0,shade.blue);
      }
    }

    // calculate max values based on viewWidth
    
    maxPos = viewWidth - 1;
    maxLen = (int)Math.floor((double)viewWidth * 0.9);
    maxVel = (double)viewWidth * LEDStrip.discoMaxVelMultiplier;
    maxAcc = maxVel * LEDStrip.discoMaxAccelMultiplier;
    maxGrow = (double)viewWidth * LEDStrip.discoMaxGrowMultiplier;
    maxGrowRate = maxGrow * LEDStrip.discoMaxGrowRateMultiplier;
    
    // set values to random value between -max and +max 
    
    velocity = ((Math.random() - 0.5) * 2) * maxVel;
    accel = ((Math.random() - 0.5) * 2) * maxAcc;
    growth = ((Math.random() - 0.5) * 2) * maxGrow;
    growthRate = ((Math.random() - 0.5) * 2) * maxGrowRate;
    startTime = Timer.getTimestamp();
    lastTime = startTime;
  }

}
