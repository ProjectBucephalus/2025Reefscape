package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import frc.robot.util.FieldUtils.DriverFieldRefs;
import frc.robot.constants.Constants.LEDStrip;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;

public class LightLayer
{

  AddressableLEDBuffer lightBuff;
  AddressableLEDBuffer shortBuff;
  final LEDPattern patternBlack = LEDPattern.solid(Color.kBlack);
  Color colorOn = LEDStrip.defaultFrontColor;
  Color colorOff = LEDStrip.defaultBackColor;
  private CommandSwerveDrivetrain s_Swerve;
  private int startLED;
  private LEDPattern display;
  public enum Mode {DRIVERFACE, WHOLESTRIP, TARGETFACE}
  public double progress;
  private Mode displayMode;
  public enum LEDType {INDIVIDUAL, PROGRESS, STATUS, POINTER}
  private LEDType displayType;
  boolean toggleMode = false;
  boolean toggleType = false;
  int statusSegments = LEDStrip.defaultStatusSegments;
  Color[] statusOn = new Color[statusSegments];
  Color[] statusOff = new Color[statusSegments];
  Color[] statusCol = new Color[statusSegments];
  Translation2d target;
  int priority=0;
  String name = "Default";
  int width = LEDStrip.viewWidth;
  boolean drawBorder = true;
  Color borderColor = LEDStrip.displayBorderColor;
  
  int i = 0; // Loop counter and temporary index values
  
  public LightLayer(CommandSwerveDrivetrain s_Swerve,String nameReq)
  {
    this.s_Swerve = s_Swerve;
    name = nameReq;
    lightBuff = new AddressableLEDBuffer(LEDStrip.lightsLen);
    shortBuff = new AddressableLEDBuffer(width);
    progress=0.3;  
    displayMode = Mode.DRIVERFACE;
    displayType = LEDType.PROGRESS;
    for (i=0; i<statusSegments; i++)
    {
      statusOff[i] = Color.kBlack;
      statusOn[i] = Color.kGold;
      statusCol[i] = Color.kBlack;
    }
    if (FieldUtils.isRedAlliance())
    {
      switch (DriverStation.getLocation().getAsInt())
      {
        case 1:target = DriverFieldRefs.driverRed1; break;
        case 2:target = DriverFieldRefs.driverRed2; break;
        case 3:target = DriverFieldRefs.driverRed3; break;
        default:target = DriverFieldRefs.driverRed1;
      }
    }
    else
    {
      switch (DriverStation.getLocation().getAsInt())
      {
        case 1:target = DriverFieldRefs.driverBlue1; break;
        case 2:target = DriverFieldRefs.driverBlue2; break;
        case 3:target = DriverFieldRefs.driverBlue3; break;
        default:target = DriverFieldRefs.driverBlue1;
      }
    }
  }

  public void setStatusSegments(int newSegments)
  {
    statusOn = new Color[newSegments];
    statusOff = new Color[newSegments];
    statusCol = new Color[newSegments];
    for (i=0; i<statusSegments; i++)
    {
      statusOff[i] = Color.kBlack;
      statusOn[i] = Color.kGold;
      statusCol[i] = Color.kBlack;
    }  
  }

  public void setProgress (double newProgress)
    {progress = newProgress;}
  

  public void setTarget (Translation2d newTarget)
    {target = newTarget;}

  public void setWidth (int newWidth)
  {
    width = newWidth;
    shortBuff = new AddressableLEDBuffer(width);
  }

  public int getWidth()
    {return width;}

  public void setPriority (int newPriority)
    {priority = newPriority;}

  public int getPriority()
    {return priority;}

  public String getName()
    {return name;}

  public void setMode (Mode newMode)
  {
    displayMode = newMode;
    if (displayMode == Mode.DRIVERFACE)
    {
      if (FieldUtils.isRedAlliance())
      {
        switch (DriverStation.getLocation().getAsInt())
        {
          case 1:target = DriverFieldRefs.driverRed1; break;
          case 2:target = DriverFieldRefs.driverRed2; break;
          case 3:target = DriverFieldRefs.driverRed3; break;
          default:target = DriverFieldRefs.driverRed1;
        }
      }
      else 
      {
        switch (DriverStation.getLocation().getAsInt())
        {
          case 1:target = DriverFieldRefs.driverBlue1; break;
          case 2:target = DriverFieldRefs.driverBlue2; break;
          case 3:target = DriverFieldRefs.driverBlue3; break;
          default:target = DriverFieldRefs.driverBlue1;
        }
      }  
    }
  }

  public void setType (LEDType newType)
    {displayType = newType;}

  public void setColor (Color front,Color back)
  {
      colorOn = front;
      colorOff = back;
  }

  public void setBorder (boolean borderState)
    {drawBorder = borderState;}

  public void setBorderColor (Color newBorder)
    {borderColor = newBorder;}

  public void setStatusColor (int index, Color on, Color off)
  {
    boolean current = (statusCol[index] == statusOn[index]);
    statusOn[index] = on;
    statusOff[index] = off;
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
    if (displayType != LEDType.INDIVIDUAL) {return false;}
    if (displayMode != Mode.WHOLESTRIP)
    {
      if (shortBuff.getLength()<num) {return false;}
      shortBuff.setRGB(num, red, green, blue);
    }
    else
    {
      if (lightBuff.getLength()<num) {return false;}
      lightBuff.setRGB(num, red, green, blue);
    }
    return true;
  }

  public void render(AddressableLEDBuffer LEDBuffer)
  {
    if (displayType == LEDType.PROGRESS)
    {
      progress = Conversions.clamp(progress,0.01, 1.0);
      display = LEDPattern.steps(Map.of(0,colorOn,progress,colorOff));
      if (displayMode == Mode.WHOLESTRIP)
      {
        display.applyTo(lightBuff);
      }
      else
      {
        display.applyTo(shortBuff);
      }
    }

    if (displayType == LEDType.STATUS)
    {
      Map<Double, Color> statDisPat = new HashMap<>();
      for (i=0; i<statusSegments; i++)
      {
        statDisPat.put(((double) i) / statusSegments, statusCol[i]);
        SmartDashboard.putNumber(name + "Status " + i, ((double) i) / statusSegments);
      }
      display = LEDPattern.steps(statDisPat);//Map.of(0,StatusCol[0],0.33,StatusCol[1],0.66,StatusCol[2]));
      if (displayMode == Mode.WHOLESTRIP)
      {
        display.applyTo(lightBuff);
      }
      else
      {
        display.applyTo(shortBuff);
      }
    }

    if (displayType == LEDType.POINTER)
    {
      if (width < LEDStrip.pointerGradientThreshold)
      {
        display = LEDPattern.solid(colorOn);
      }
      else
      {
        display = LEDPattern.gradient(GradientType.kContinuous, colorOff,colorOn);
      }
      if (displayMode == Mode.WHOLESTRIP)
      {
        display.applyTo(lightBuff);
      }
      else
      {
        display.applyTo(shortBuff);
      }
    }

    if (displayMode != Mode.WHOLESTRIP)
    {
      patternBlack.applyTo(lightBuff);
      double robotAngle = s_Swerve.getState().Pose.getRotation().getDegrees();
      double targetAngle = target.minus(s_Swerve.getState().Pose.getTranslation()).getAngle().getDegrees();
      startLED = (int)Math.floor((robotAngle - targetAngle)/LEDStrip.degreesPerLED) + LEDStrip.startOffset - (width/2);

      while ((startLED >= LEDStrip.lightsLen)||(startLED < 0))
      {
        if (startLED >= LEDStrip.lightsLen) {startLED -= LEDStrip.lightsLen;}
        if (startLED < 0) {startLED += LEDStrip.lightsLen;}
      }

      for (i = 0; i < width; i++)
      {
        int j = startLED + i;
        if (j >= LEDStrip.lightsLen) {j -= LEDStrip.lightsLen;}
        if (shortBuff.getLED(i) != Color.kBlack)
        {
          lightBuff.setLED(j, shortBuff.getLED(i));
        }
      }

      if (drawBorder)
      {
        i = startLED - 1;
        if (i < 0) {i+=LEDStrip.lightsLen;}
        lightBuff.setLED(i, borderColor);
        i = startLED + width;
        if (i >= LEDStrip.lightsLen) {i -= LEDStrip.lightsLen;}
        lightBuff.setLED(i, borderColor);
      }
    }
    int counter = 0;
    for (i=0; i<LEDBuffer.getLength(); i++)
    {
      if (!(lightBuff.getLED(i).equals(Color.kBlack)))
      {
        LEDBuffer.setLED(i, lightBuff.getLED(i));
        counter++;
      }
    }
    SmartDashboard.putNumber(name + ": Not Black LED's", (double)counter);
//     Lights.setData(Light_Buff);
  }
}