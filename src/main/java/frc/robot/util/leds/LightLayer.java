package frc.robot.util.leds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;
import frc.robot.util.FieldUtils.DriverFieldRefs;
import frc.robot.util.SD;
import frc.robot.constants.Constants.LEDStrip;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
/**
 * Helper class for LEDRenderer class, contains all data and render logic for a single display element (layer)
 * 
 * <p> Uses wpilibj.util Color library 
 */
public class LightLayer
{

  AddressableLEDBuffer tempBuff; // internal LED buffers for operations and/or state memory (individual mode)
  final LEDPattern patternBlack = LEDPattern.solid(Color.kBlack);  //black pattern constant useful for wiping buffers
  Color colorOn = LEDStrip.defaultFrontColor; // front or 'on' color for the layer
  Color colorOff = LEDStrip.defaultBackColor; // back or 'off' color for the layer
  private CommandSwerveDrivetrain s_Swerve; // need a refernece to the drivetrain object to retrieve/calculate robot angles
  private int startLED;  // used to calculate the starting LED when rendering
  private int startSegment; // starting position within defined segment for the layer.
  private LEDPattern display; // LEDPatternobject used to generate certain display modes
  public double progress; // the state of the PROGRESS display type (the portion of the bar that is 'front' color, as double from 0 - 1)
  private boolean reversed; // if true, the layer will be drawn reversed (useful for vertical displays, where the strip runs up and down an object)
  public enum Mode {DRIVERFACE, WHOLESTRIP, TARGETFACE, STATICSEGMENT, NEARSEGMENT, FARSEGMENT}
  private Mode displayMode; // variable using the Mode enum to set how/where this layer will be displayed
  public enum LayerType {INDIVIDUAL, PROGRESS, STATUS, POINTER, DISCO, SCROLLER, FLAME, SOLID, ALTERNATING}
  private LayerType displayType; // variable using the LEDType enum, to set which type of layer this is.
  int segments = LEDStrip.defaultStatusSegments; // numberof segments for the STATUS displayType.
  Color[] statusOn = new Color[segments];  // Arrays to store per segment on/off/current Colors for the STATUS displaytype.
  Color[] statusOff = new Color[segments];
  Color[] statusCol = new Color[segments];
  Translation2d target; // the target location, as Traslation2d from field origin, that displays should be facing/ pointing towards.
  double robotAngle = 0; // used in the render method to calculate the robots current angle from x axis.
  double targetAngle = 0; // used in the render method to calculate the current angle to the x axis of the vector to the target from the robot.
  int priority=0; // display priority of this layer, higher priorities will be drawn on top.
  String name = "Default"; // name of this layer, used for debugging and layer search / deletion, should be unique, but this is not enforced.
  int width = LEDStrip.viewWidth; // number of LED's this layer uses
  boolean drawBorder = true; // if true, will draw a single LED of borderColor each side of this layer
  Color borderColor = LEDStrip.displayBorderColor; // Color of the border dots.
  ArrayList<DiscoLayer> discoQueue = new ArrayList<DiscoLayer>(); // used to generate the display for the 'disco' displayType.
  private double period; // number of seconds for animated effects to go through a cycle
  ArrayList<Integer> ashLocations; //list of black spots for flame effect
  private double lastTime;
  private double currTime;
  private int state;
  
  /**
   * default constructor, takes drivetrain refernce object and 'name' string and sets defaults for other values
   */
  public LightLayer(CommandSwerveDrivetrain s_Swerve,String nameReq)
  {
    this.s_Swerve = s_Swerve;
    name = nameReq;
    tempBuff = new AddressableLEDBuffer(width);
    progress=0.3;  
    displayMode = Mode.DRIVERFACE;
    displayType = LayerType.PROGRESS;
    //set default STATUS values
    for(int i=0; i<segments; i++)
    {
      statusOff[i] = colorOff;
      statusOn[i] = colorOn;
      statusCol[i] = colorOff;
    }
    // Get driverstation location from the driverstation / FMS and set target for driverfacing.
    if (FieldUtils.isRedAlliance())
    {
      switch (FieldUtils.getDriverLocation())
      {
        case 1:target = DriverFieldRefs.driverRed1; break;
        case 2:target = DriverFieldRefs.driverRed2; break;
        case 3:target = DriverFieldRefs.driverRed3; break;
        default:target = DriverFieldRefs.driverRed1;
      }
    }
    else
    {
      switch (FieldUtils.getDriverLocation())
      {
        case 1:target = DriverFieldRefs.driverBlue1; break;
        case 2:target = DriverFieldRefs.driverBlue2; break;
        case 3:target = DriverFieldRefs.driverBlue3; break;
        default:target = DriverFieldRefs.driverBlue1;
      }
    }

    SD.IO_LED_BRIGHTNESS.init();
  }

  public void setSegments(int newSegments)
  {
    /**
     * sets the number of segments for the STATUS and SCROLLER displayTypes.
     * 
     * <p> Note this operation is destructive of any currrent STATUS, all segments will be reset to 'off'
     */
    // redefine the arrays to new size
    segments = newSegments;
    statusOn = new Color[segments];
    statusOff = new Color[segments];
    statusCol = new Color[segments];
    // set default values
    for(int i=0; i<segments; i++)
    {
      statusOff[i] = colorOff;
      statusOn[i] = colorOn;
      statusCol[i] = colorOff;
    }
  }

  public void setProgress (double newProgress)
    /**
     * Set the progress value for PROGRESS displayType
     * <p> valid values 0.0 - 1.0, values outside this range will be clamped.
     */
    {progress = newProgress;}

  public void setReversed (boolean reverse)
  {
    /**
     * Set whether the display for this layer should be reversed
     */
    if ((reversed != reverse) && (displayType == LayerType.INDIVIDUAL))
    {
      // if the value is different, and the displayType is INDIVIDUAL, flip the current buffer.
      Color tempColor;
      for(int i = 0; i < (width / 2); i++)
      {
        tempColor = tempBuff.getLED(i);
        tempBuff.setLED(i, tempBuff.getLED((width - 1) - i));
        tempBuff.setLED((width - 1) - i, tempColor);
      }
    }  
    reversed = reverse;
  }

  public void setTarget (Translation2d newTarget)
    /**
     * set a new target location for the layer.
     */
    {target = newTarget;}

  public void setStart(int LEDStart)
    /**
     * Set the starting location (LED) for the layer
     * <p> Note this value is relative to the start of the segment for Near/Far segment diplayModes
     * and is unused for target/driver facing and wholestrip modes
     */
    {startSegment = LEDStart;}

  public boolean setWidth (int newWidth)
  {
    /**
     * set new width (number of LED's) for this layer.
     * <p> Will return false and not change width if the displaymode is NEARSEGMENT or FARSEGMENT,
     * and the requested width would overrun the segment, or if it is longer than the length of the LED's
     */
    if ((displayMode == Mode.FARSEGMENT) || (displayMode == Mode.NEARSEGMENT))
    {
      // check if new width would overrun the display area.
      int end = startSegment + newWidth;
      //if (end >= LEDStrip.lightsLen) { end -= LEDStrip.lightsLen; }
      if (end < (Math.max((LEDStrip.stbdLEDsEnd - LEDStrip.stbdLEDsStart),(LEDStrip.portLEDsEnd-LEDStrip.portLEDsStart))))
      {
        width = newWidth;
        tempBuff = new AddressableLEDBuffer(width);
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      // in all other modes width can go over the end of the strip with no problems,
      //just must be less than the length of the strip(s)

      if (newWidth <= LEDStrip.lightsLen)
      {
        width = newWidth;
        tempBuff = new AddressableLEDBuffer(width);
        return true;
      }
      else
      {
        return false;
      }
    }
  }

  public int getWidth()
    /**
     * getter for the width variable.
     */

    {return width;}

  public void setPriority (int newPriority)
    /**
     * set new value for the layer priority.
     * <p> Higher priority layers will be drawn on top of others.
     * <p> A layer with negative priority will not be drawn.
     */

    {priority = newPriority;}

  public int getPriority()
    /**
     * Getter for the priority variable.
     * <p> Implemented mainly to make the class sortable by priority in the LEDRenderer class.
     */

    {return priority;}

  public String getName()
    /**
     * Getter for the name variable, used to locate a particular layer in the renderQueue in LEDRenderer
     */

    {return name;}

  public void setMode (Mode newMode)
  {
    /**
     * Sets the displayMode for the layer
     */

    displayMode = newMode;
    if (displayMode == Mode.DRIVERFACE)
    {

      // get driver location from the driverstation / FMS
      if (FieldUtils.isRedAlliance())
      {
        switch (FieldUtils.getDriverLocation())
        {
          case 1:target = DriverFieldRefs.driverRed1; break;
          case 2:target = DriverFieldRefs.driverRed2; break;
          case 3:target = DriverFieldRefs.driverRed3; break;
          default:target = DriverFieldRefs.driverRed1;
        }
      }
      else 
      {
        switch (FieldUtils.getDriverLocation())
        {
          case 1:target = DriverFieldRefs.driverBlue1; break;
          case 2:target = DriverFieldRefs.driverBlue2; break;
          case 3:target = DriverFieldRefs.driverBlue3; break;
          default:target = DriverFieldRefs.driverBlue1;
        }
      }  
    }
    else if (newMode == Mode.WHOLESTRIP)
    {
      startLED = 0;
      width = LEDStrip.lightsLen;
      tempBuff = new AddressableLEDBuffer(width);
    }
  }

  public void setPeriod (double newPeriod)
  /**
   * Sets the period of effects in seconds (time to repeat the pattern, or scroll the width of the segment)
   */
  {period = newPeriod;}

  public void setType (LayerType newType)
    /**
     * Sets the displayType for the layer.
     */

    {displayType = newType;}

  public void setColor (Color front,Color back)
  {
    /**
     * Set the front/on and back/off Colors for the layer.
     * 
     * <p> Note that if called while in STATUS display type will overwrite individual status colours.
     */

    colorOn = front;
    colorOff = back;
    if (displayType == LayerType.STATUS)
    {
      for(int i = 0; i < segments; i++)
      {
        boolean tempStatus = (statusCol[i].equals(statusOn[i]));
        statusOn[i] = colorOn;
        statusOff[i] = colorOff;
        if (tempStatus)
          {statusCol[i] = colorOn;}
        else
          {statusCol[i] = colorOff;}
      }
    }
  }

  public void setBorder (boolean borderState)
    /**
     * Set whether to draw a border for this layer (one LED on either end, in a different Color)
     */

    {drawBorder = borderState;}

  public void setBorderColor (Color newBorder)
    /**
     * Set the border Color for this layer.
     */

    {borderColor = newBorder;}

  public void setStatusColor (int index, Color on, Color off)
  {
    /**
     * Set the on / off Colors for a STATUS indicator.
     */

    // temporarily store whether the indicator is currently off or on.
    boolean current = (statusCol[index].equals(statusOn[index]));

    // store new colors
    statusOn[index] = on;
    statusOff[index] = off;

    // set the current state to what it was before 
    if (current)
    {
      statusCol[index] = on;
    }
    else
    {
      statusCol[index] = off;
    }
  }

  public void setStatus (int index, boolean newStatus)
  {
    /**
     * Set a STATUS indicator to off or on.
     */

    if (newStatus)
    {
      statusCol[index] = statusOn[index];
    }
    else
    {
      statusCol[index] = statusOff[index];
    }
  }
    
  public boolean setLight (int num, int red, int green, int blue)
  {
    /**
     * Set an individual LED to Color (R,G,B)
     * <p> Will return false if displayType is not INDIVIDUAL, or if index is greater than width-1.
     */

    if (displayType != LayerType.INDIVIDUAL) {return false;}
    if (tempBuff.getLength()<num) {return false;}
    if (reversed)
    {
      tempBuff.setRGB((width - 1) - num, red, green, blue);
    }
    else
    {
      tempBuff.setRGB(num, red, green, blue);
    }
    return true;
  }

  public boolean setLight (int num, Color shade)
  {
    /**
     * Set an individual LED to Color
     * <p> Will return false if displayType is not INDIVIDUAL, or if index is greater than width-1.
     */

    if (displayType != LayerType.INDIVIDUAL) {return false;}
    if (tempBuff.getLength()<num) {return false;}
    if (reversed)
    {
      tempBuff.setLED((width - 1) - num, shade);
    }
    else
    {
      tempBuff.setLED(num, shade);
    }
    return true;
  }

  public Color adjustBrightness(Color inputColour)
  {
    double brightness = SD.IO_LED_BRIGHTNESS.get();
    return new Color
    (
      inputColour.red   * brightness,
      inputColour.green * brightness,
      inputColour.blue  * brightness
    );
  }

  public void render(AddressableLEDBuffer LEDBuffer)
  {
    /**
     * Will draw the current state of this layer onto an AddressableLEDBuffer representing the whole LED Strip
     * 
     * <p> while intended to only be called from LEDRenderer, there may be other use cases.
     * 
     * <p> It is expected, but not checked, that LEDBuffer.size() = Constants.LEDStrip.lightsLen
     * a larger buffer will work, but may produce unexpected output, this has not been checked
     * a buffer size smaller than Constants.LEDStrip.lightsLen will result in an array index out of bounds error.
     */

     if (displayType == LayerType.PROGRESS)
    {

      // check progress variable is not out of bounds.
      progress = MathUtil.clamp(progress,0.01, 1.0);

      // use the LEDPattern object to build a display that is progress% the front/on color.
      display = LEDPattern.steps(Map.of(0,colorOn,progress,colorOff));
      if (reversed) {display = display.reversed();}
      display.applyTo(tempBuff);
    }
    
    if (displayType == LayerType.SCROLLER)
    {

      // build a map of starting position and colours for each element
      Map<Double, Color> statDisPat = new HashMap<>();
      for(int i=0; i<segments*2; i+=2)
      {
        statDisPat.put(((double) i) / (segments * 2), colorOn);
        statDisPat.put(((double) i + 1) / (segments * 2), colorOff);
      }

      // use the LEDPattern object again to apply the map to the buffer
      display = LEDPattern.steps(statDisPat);
      double timeFactor = Timer.getTimestamp() * period;
      timeFactor = timeFactor - Math.floor(timeFactor);
      display = display.offsetBy((int)(width * timeFactor));
      if (reversed) {display = display.reversed();}
      display.applyTo(tempBuff);
    }

    if (displayType == LayerType.FLAME)
    {
      display = LEDPattern.gradient(GradientType.kDiscontinuous, colorOn, colorOff);
      display = display.mask(LEDPattern.progressMaskLayer(Math::random));
      if (reversed) {display = display.reversed();}
      display.applyTo(tempBuff);
      currTime = Timer.getTimestamp();
      if (currTime - lastTime > period)
      {
        if ((ashLocations.size() < 5) && (Math.random() > 0.8))
          { ashLocations.add(0); }
        if (ashLocations.size() > 0)
        {
          for(int i = 0; i < ashLocations.size(); i++)
          {
            if (reversed)
            {
              tempBuff.setRGB(width - (i + 1), 1, 0, 0);
            }
            else
            {
              tempBuff.setRGB(i, 1, 0, 0);
            }
            ashLocations.set(i, ashLocations.get(i) + 1);
            if (ashLocations.get(i) > width - 1)
              { ashLocations.remove(i); }
          }
        }
        lastTime = currTime;
      }
    }

    if (displayType == LayerType.SOLID)
    {
      display = LEDPattern.solid(colorOn);
      display.applyTo(tempBuff);
    }

    if (displayType == LayerType.ALTERNATING)
    {
      patternBlack.applyTo(tempBuff);
      currTime = Timer.getTimestamp();
      if (currTime - lastTime > period)
      {
        if (state != 0)
        {
          state = 0;
        }
        else
        {
          state = 1;
        }
        lastTime = currTime;
      }
      if (state == 0)
      {
        for(int i = 0; i < width; i += 2)
        {
          tempBuff.setLED(i, colorOn);
        }
      }
      else
      {
        for(int i = 1; i < width; i += 2)
        {
          tempBuff.setLED(i, colorOn);
        }
      }
    }


    if (displayType == LayerType.STATUS)
    {

      // build a map of starting position (as %) and colours foreach STATUS element
      Map<Double, Color> statDisPat = new HashMap<>();
      for(int i=0; i<segments; i++)
      {
        statDisPat.put(((double) i) / segments, statusCol[i]);
        //SmartDashboard.putNumber(name + "Status " + i, ((double) i) / statusSegments);
      }

      // use the LEDPattern object again to applythe map to the appropriate buffer
      display = LEDPattern.steps(statDisPat);//Map.of(0,StatusCol[0],0.33,StatusCol[1],0.66,StatusCol[2]));
      if (reversed) {display = display.reversed();}
      display.applyTo(tempBuff);
    }

    if (displayType == LayerType.POINTER)
    {

      // if the pointer is below a certain size there is no point getting fancy, theres not enough LED's to see it
      // if its wide enough though, we can have a fade in/out effect
      if (width < LEDStrip.pointerGradientThreshold)
      {
        display = LEDPattern.solid(colorOn);
      }
      else
      {
        display = LEDPattern.gradient(GradientType.kContinuous, colorOff,colorOn);
      }

      // reversing this pattern should do nothing, since it's symetrical, but this is here anyway for futureproofing.
      if (reversed) {display = display.reversed();}
      display.applyTo(tempBuff);
    }

    if (displayType == LayerType.DISCO)
    {
      // the disco displaytype is a set of random blocks of moving colour, meant to be attention grabbing
      // it is implemented with a layer system similar to the main LEDRenderer, except they can self update
      // various attributes such as size,position and speed.
      //

      // first off, if there are too few layers, or randomly if the number is between min and max, add a new one.
      if ((discoQueue.size() < LEDStrip.discoMin) || 
          ((discoQueue.size() < LEDStrip.discoMax) && (Math.random() > 0.8)))
      {
        discoQueue.add(new DiscoLayer(width));
      }

      // then randomly, but a bit more often if the queue is at max size, remove a layer.
      if (((discoQueue.size() > LEDStrip.discoMin) && (Math.random() > 0.9)) || 
          ((discoQueue.size() == LEDStrip.discoMax) && (Math.random() > 0.5)))
      {
        discoQueue.remove((int)Math.floor(Math.random()*(discoQueue.size()-1)));
      }

      // iterate through the queue
      for (DiscoLayer disco : discoQueue)
      {
        // call the update method to refresh layer attributes.
        disco.update();

        // then copy the layer colour to the appropriate area of the buffer
        for(int i=0; i < disco.getLength(); i++)
        {
          int bufferIndex = startLED + disco.getStartLED() + i;
//          if (bufferIndex >= width) {bufferIndex-=width;}
          bufferIndex = Conversions.wrap(bufferIndex, startLED, startLED + width - 1);
          tempBuff.setLED(bufferIndex, disco.shade);
        }

        // layers will die of old age randomly between AgeLimit and AgeLimit*2 seconds
        if ((((double)disco.age / LEDStrip.discoAgeLimit) + Math.random()) > 2)
        {
          discoQueue.remove(disco);
        }
      }
    }

    if ((displayMode != Mode.WHOLESTRIP) && (displayMode != Mode.STATICSEGMENT))
    {
      // if we are in a displayMode / displayType combination where it is relevant, calculate the robot and target angles
      robotAngle = s_Swerve.getState().Pose.getRotation().getDegrees();
      targetAngle = target.minus(s_Swerve.getState().Pose.getTranslation()).getAngle().getDegrees();
    }

    // startLED calculations for various modes.
    if (displayMode == Mode.STATICSEGMENT)
      {startLED = startSegment;}

    if ((displayMode == Mode.DRIVERFACE) || (displayMode == Mode.TARGETFACE))
    {
      startLED = (int)Math.floor((robotAngle - targetAngle)/LEDStrip.degreesPerLED) + LEDStrip.startOffset - (width/2);
    }

    if (displayMode == Mode.NEARSEGMENT)
    {
      if ((robotAngle - targetAngle) < 0)
        { startLED = LEDStrip.stbdLEDsStart + startSegment; }
      else
        { startLED = LEDStrip.portLEDsStart + startSegment; }
    }

    if (displayMode == Mode.FARSEGMENT)
    {
      if ((robotAngle - targetAngle) > 0)
        { startLED = LEDStrip.stbdLEDsStart + startSegment; }
      else
        { startLED = LEDStrip.portLEDsStart + startSegment; }
    }

    // Then, copy the internal/temporary buffer onto the output buffer
    // First check startLED calculations haven't resulted in something out of bounds
    //while ((startLED >= LEDStrip.lightsLen)||(startLED < 0))
    //{
    //  if (startLED >= LEDStrip.lightsLen) {startLED -= LEDStrip.lightsLen;}
    //  if (startLED < 0) {startLED += LEDStrip.lightsLen;}
    //}
    startLED = Conversions.wrap(startLED, 0, LEDStrip.lightsLen);

    // copy the buffer onto the output
    for(int inputIndex = 0; inputIndex < width; inputIndex++)
    {
      int outputIndex = startLED + inputIndex;
      outputIndex = Conversions.wrap(outputIndex, 0, LEDStrip.lightsLen-1);
//      if (j >= LEDStrip.lightsLen) {j -= LEDStrip.lightsLen;}
      if (!(tempBuff.getLED(inputIndex).equals(Color.kBlack)))
      {
        LEDBuffer.setLED(outputIndex, adjustBrightness(tempBuff.getLED(inputIndex)));
      }
    }

    //draw border if set
    if (drawBorder)
    {
      int i = startLED - 1;
      if (i < 0) {i += LEDStrip.lightsLen;}
      LEDBuffer.setLED(i, adjustBrightness(borderColor));
      i = startLED + width;
      if (i >= LEDStrip.lightsLen) {i -= LEDStrip.lightsLen;}
      LEDBuffer.setLED(i, adjustBrightness(borderColor));
    }
  }
}