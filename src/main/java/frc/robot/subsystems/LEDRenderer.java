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

  AddressableLED lights;
  AddressableLEDBuffer lightBuffer;
  final LEDPattern patternBlack = LEDPattern.solid(Color.kBlack);
//    private CommandSwerveDrivetrain s_Swerve;
  ArrayList<LightLayer> renderQueue = new ArrayList<LightLayer>();
  LightLayer layer;
  
  int i = 0; // Loop counter and temporary index values
  
  public LEDRenderer(/*CommandSwerveDrivetrain s_Swerve*/)
  {
//    this.s_Swerve = s_Swerve;
  lights = new AddressableLED(LEDStrip.LEDPWMPort);
  lightBuffer = new AddressableLEDBuffer(LEDStrip.lightsLen);
  lights.setLength(LEDStrip.lightsLen);
  lights.start();
  }

  public void addLayer (LightLayer newLayer)
    {renderQueue.add(newLayer);}

  public LightLayer getLayer (int index)
    {return renderQueue.get(index);}

  public LightLayer getLayer (String name)
  {
    for (LightLayer layer : renderQueue)
    {
      if (layer.getName() == name)
      {
        return layer;
      }
    }
    return null;
  }

  public void removeLayer (String nameToGo)
    {renderQueue.removeIf(a -> a.getName() == nameToGo);}

  public void removeLayer (int index) 
    {renderQueue.remove(index);}

  public String[] listLayers()
  {
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
    patternBlack.applyTo(lightBuffer);
    renderQueue.sort(Comparator.comparing(LightLayer::getPriority));
    for (LightLayer layer : renderQueue) 
    {
      layer.render(lightBuffer);
    }
    lights.setData(lightBuffer);
  }
}