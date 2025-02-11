package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Rumbler extends SubsystemBase 
{
  private CommandXboxController rumbleDriver;
  private CommandXboxController rumbleCopilot;

  // Arraylist "queue" for rumble requests for each rumble motor (DriverLeft-DL, DriverRight-DR, CopilotLeft-CL, CopilotRight-CR)
  private ArrayList<String> drRequest = new ArrayList<String>();
  private ArrayList<String> dlRequest = new ArrayList<String>();
  private ArrayList<String> crRequest = new ArrayList<String>();
  private ArrayList<String> clRequest = new ArrayList<String>();
  // Enum used to specify queue in add and remove methods
  public enum Sides{DRIVER_RIGHT, DRIVER_LEFT, COPILOT_RIGHT, COPILOT_LEFT}
  private double driverStrength;
  private double copilotStrength;

  public Rumbler(CommandXboxController driver, CommandXboxController copilot)
  {
    SmartDashboard.putNumber("Driver Rumble", Constants.Rumbler.driverDefault);
    SmartDashboard.putNumber("Copilot Rumble", Constants.Rumbler.copilotDefault);

    // could drop the getHID method as setrumble has been added to the CommandXBoxController class in 2025, but this still works.
    rumbleDriver = driver;
    rumbleCopilot = copilot;  
    // Check if smartdashboard has existing settings for driver and copilot rumble strength, and put defaults if not.
    driverStrength = SmartDashboard.getNumber("Driver Rumble", Constants.Rumbler.driverDefault);
    copilotStrength = SmartDashboard.getNumber("Copilot Rumble", Constants.Rumbler.copilotDefault);
  } 

  public boolean addRequest(Sides queue, String requestID)
  {
    // requestID is a unique string to identify the rumble request
    // this request will stay active until a matching removeRequest is received.
    switch (queue)
    {
      case DRIVER_RIGHT:
        if(!drRequest.contains(requestID))
          {return drRequest.add(requestID);}
        break;
      case DRIVER_LEFT:
        if(!dlRequest.contains(requestID))
          {return dlRequest.add(requestID);}
        break;
      case COPILOT_RIGHT:
        if(!crRequest.contains(requestID))
          {return crRequest.add(requestID);}
        break;
      case COPILOT_LEFT:
        if(!clRequest.contains(requestID))
          {return clRequest.add(requestID);}
        break;
      default:
        return false;
    }
    return false;
  } 

  public boolean removeRequest(Sides queue, String requestID)
  {
    switch (queue)
    {
      case DRIVER_RIGHT:
        return drRequest.remove(requestID);
      case DRIVER_LEFT:
        return dlRequest.remove(requestID);
      case COPILOT_RIGHT:
        return crRequest.remove(requestID);
      case COPILOT_LEFT:
        return clRequest.remove(requestID);
      default:
        return false;
    }    
  }

  @Override
  public void periodic()
  {
    // check for chages to rumble stregnths in smartdashboard, and update.
    driverStrength = SmartDashboard.getNumber("Driver Rumble", Constants.Rumbler.driverDefault);
    copilotStrength = SmartDashboard.getNumber("Copilot Rumble", Constants.Rumbler.copilotDefault);
    // if there are any active requests in the queue for a rumble motor, rumble, otherwise stop.
    if (drRequest.size() > 0)
      {rumbleDriver.setRumble(GenericHID.RumbleType.kRightRumble, driverStrength);}
    else
      {rumbleDriver.setRumble(GenericHID.RumbleType.kRightRumble, 0);}
    if(dlRequest.size() > 0)
      {rumbleDriver.setRumble(GenericHID.RumbleType.kLeftRumble,driverStrength);}
    else
      {rumbleDriver.setRumble(GenericHID.RumbleType.kLeftRumble,0);}
    if(crRequest.size() > 0)
      {rumbleCopilot.setRumble(RumbleType.kRightRumble,copilotStrength);}
    else
      {rumbleCopilot.setRumble(RumbleType.kRightRumble,0);}
    if(clRequest.size() > 0)
      {rumbleCopilot.setRumble(RumbleType.kLeftRumble,copilotStrength);}
    else
      {rumbleCopilot.setRumble(RumbleType.kLeftRumble,0);}
    // put queue contents to dashboard, for debugging / verification.
    SmartDashboard.putString("DriverRight Rumble Queue",drRequest.toString());
    SmartDashboard.putString("CopilotLeft Rumbler Queue",clRequest.toString());
    SmartDashboard.putString("DriverLeft Rumble Queue",dlRequest.toString());
    SmartDashboard.putString("CopilotRight Rumble Queue",crRequest.toString());
  }
}
