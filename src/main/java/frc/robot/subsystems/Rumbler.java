package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SD;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    SD.IO_RUMBLE_D.init();
    SD.IO_RUMBLE_C.init();

    rumbleDriver = driver;
    rumbleCopilot = copilot;  
    // Check if smartdashboard has existing settings for driver and copilot rumble strength, and put defaults if not.
    driverStrength = SD.IO_RUMBLE_D.get();
    copilotStrength = SD.IO_RUMBLE_C.get();
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

  public Command requestCommand(boolean addRequest, Sides queue, String requestID)
  {
    if (addRequest) 
    {
      return Commands.runOnce(() -> addRequest(queue, requestID));   
    }
    else
    {
      return Commands.runOnce(() -> removeRequest(queue, requestID)); 
    }
  }

  public Command whileTriggerRequestCommand(Sides queue, String requestID)
  {
    return Commands.startEnd(() -> addRequest(queue, requestID), () -> removeRequest(queue, requestID));
  }

  public Command timedRequestCommand(Sides queue, String requestID, double durationSeconds)
  {
    return 
    Commands.sequence
    (
      requestCommand(true, queue, requestID),
      Commands.waitSeconds(durationSeconds),
      requestCommand(false, queue, requestID)
    );
  }

  @Override
  public void periodic()
  {
    // check for chages to rumble stregnths in smartdashboard, and update.
    driverStrength  = SD.IO_RUMBLE_D.get();
    copilotStrength = SD.IO_RUMBLE_C.get();
    // if there are any active requests in the queue for a rumble motor, rumble, otherwise stop.
    rumbleDriver.setRumble(RumbleType.kRightRumble, drRequest.isEmpty() ? 0 : driverStrength);
    rumbleDriver.setRumble(RumbleType.kLeftRumble, dlRequest.isEmpty() ? 0 : driverStrength);
    rumbleCopilot.setRumble(RumbleType.kRightRumble, crRequest.isEmpty() ? 0 : copilotStrength);
    rumbleCopilot.setRumble(RumbleType.kLeftRumble, clRequest.isEmpty() ? 0 : copilotStrength);

    // put queue contents to dashboard, for debugging / verification.
    SD.RUMBLE_D_R.put(drRequest.toString());
    SD.RUMBLE_D_L.put(dlRequest.toString());
    SD.RUMBLE_C_R.put(crRequest.toString());
    SD.RUMBLE_C_L.put(clRequest.toString());
  }
}
