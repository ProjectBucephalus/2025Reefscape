package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.LEDStrip;
import frc.robot.util.LightLayer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;
import java.util.Comparator;

public class LEDRenderer extends SubsystemBase
{
/* LEDRenderer class, maintains and orders the list of LightLayer objects that need*/
/* to be rendered to the LED strip */

  AddressableLED lights;
  AddressableLEDBuffer lightBuffer;
  final LEDPattern patternBlack = LEDPattern.solid(Color.kBlack); //Useful to wipe the buffer before each render pass
//    private CommandSwerveDrivetrain s_Swerve;
  ArrayList<LightLayer> renderQueue = new ArrayList<LightLayer>(); //List of layers to be rendered
  LightLayer layer; //temporary layer object
  
  int i = 0; // Loop counter and temporary index values
  
  public LEDRenderer()
  {
    // initial setup, attach the PWM port, define the buffer length, and the length of the strip,
    // and start the LED driver.

  lights = new AddressableLED(LEDStrip.LEDPWMPort);
  lightBuffer = new AddressableLEDBuffer(LEDStrip.lightsLen);
  lights.setLength(LEDStrip.lightsLen);
  lights.start();
  }

  public void addLayer (LightLayer newLayer) // add a layer object into the queue
    {renderQueue.add(newLayer);}

  public LightLayer getLayer (int index)
  {
    // retrieve a layer object by index into the queue, returns null if index out of bounds
    if (index < renderQueue.size())
    {
      return renderQueue.get(index);
    }
    else
    {
      return null;
    }
  }

  public LightLayer getLayer (String name)
  {
    // retrieve a layer object in the queue by name, returns null if not found
    for (LightLayer layer : renderQueue)
    {
      if (layer.getName() == name)
      {
        return layer;
      }
    }
    return null;
  }

  public void removeLayer (String nameToGo) // remove layer from the queue by name
    {renderQueue.removeIf(a -> a.getName() == nameToGo);}

  public void removeLayer (int index) // remove layer from the queue by index
  {
    if (index < renderQueue.size())
    {
      renderQueue.remove(index);
    }
  }

  public String[] listLayers()
  {
    // returns string array with the names of layers currently in the queue
    String[] names = new String[renderQueue.size()];
    for (i=0; i < renderQueue.size(); i++)
    {
      names[i] = renderQueue.get(i).getName();
    }
    return names;
  }

  @Override
  public void periodic() 
  {
    // wipe the buffer
    patternBlack.applyTo(lightBuffer);
    // sort the queue by layer priority
    renderQueue.sort(Comparator.comparing(LightLayer::getPriority));
    // call each layers render() method to draw to the buffer
    for (LightLayer layer : renderQueue) 
    {
      layer.render(lightBuffer);
    }
    // transfer the layered buffer to the LED Driver
    lights.setData(lightBuffer);
  }
}