package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Rumbler extends SubsystemBase 
{
    private CommandXboxController rumbleDriver;
    private CommandXboxController rumbleCodriver;

    // Arraylist "queue" for rumble requests for each rumble motor (DriverLeft-DL, DriverRight-DR, CoDriverLeft-CL, CoDriverRight-CR)
    private ArrayList<String> DR_Request = new ArrayList<String>();
    private ArrayList<String> DL_Request = new ArrayList<String>();
    private ArrayList<String> CR_Request = new ArrayList<String>();
    private ArrayList<String> CL_Request = new ArrayList<String>();
    // Enum used to specify queue in add and remove methods
    public enum Sides{DriverRight, DriverLeft, CoDriverRight, CoDriverLeft}
    private double driverStrength;
    private double coDriverStrength;

    public Rumbler (CommandXboxController driver, CommandXboxController coDriver)
    {
        // could drop the getHID method as setrumble has been added to the CommandXBoxController class in 2025, but this still works.
        rumbleDriver = driver;
        rumbleCodriver = coDriver;  
        // Check if smartdashboard has existing settings for driver and codriver rumble strength, and put defaults if not.
        driverStrength = SmartDashboard.getNumber("Driver Max Rumble", Constants.Rumbler.driver_Default);
        coDriverStrength = SmartDashboard.getNumber("CoDriver Max Rumble", Constants.Rumbler.coDriver_Default);
        SmartDashboard.putNumber("Driver Rumble", driverStrength);
        SmartDashboard.putNumber("CoDriver Rumble", coDriverStrength);
    } 

    public boolean addRequest(Sides queue, String requestID)
    {
        // requestID is a unique string to identify the rumble request
        // this request will stay active until a matching removeRequest is received.
        switch(queue)
        {
            case DriverRight:
                if(!(DR_Request.contains(requestID)))
                    {return DR_Request.add(requestID);}
                break;
            case DriverLeft:
                if(!(DL_Request.contains(requestID)))
                    {return DL_Request.add(requestID);}
                break;
            case CoDriverRight:
                if(!(CR_Request.contains(requestID)))
                    {return CR_Request.add(requestID);}
                break;
            case CoDriverLeft:
                if(!(CL_Request.contains(requestID)))
                    {return CL_Request.add(requestID);}
                break;
            default:
                return false;
        }
        return false;
    } 

    public boolean removeRequest(Sides queue, String requestID)
    {
        switch(queue)
        {
            case DriverRight:
                return DR_Request.remove(requestID);
            case DriverLeft:
                return DL_Request.remove(requestID);
            case CoDriverRight:
                return CR_Request.remove(requestID);
            case CoDriverLeft:
                return CL_Request.remove(requestID);
            default:
                return false;
        }    
    }

    @Override
    public void periodic()
    {
        // check for chages to rumble stregnths in smartdashboard, and update.
        driverStrength = SmartDashboard.getNumber("Driver Max Rumble",0);
        coDriverStrength = SmartDashboard.getNumber("CoDriver Max Rumble",0);
        // if there are any active requests in the queue for a rumble motor, rumble, otherwise stop.
        if (DR_Request.size() > 0)
            {rumbleDriver.setRumble(GenericHID.RumbleType.kRightRumble, driverStrength);}
        else
            {rumbleDriver.setRumble(GenericHID.RumbleType.kRightRumble, 0);}
        if(DL_Request.size() > 0)
            {rumbleDriver.setRumble(GenericHID.RumbleType.kLeftRumble,driverStrength);}
        else
            {rumbleDriver.setRumble(GenericHID.RumbleType.kLeftRumble,0);}
        if(CR_Request.size() > 0)
            {rumbleCodriver.setRumble(RumbleType.kRightRumble,coDriverStrength);}
        else
            {rumbleCodriver.setRumble(RumbleType.kRightRumble,0);}
        if(CL_Request.size() > 0)
            {rumbleCodriver.setRumble(RumbleType.kLeftRumble,coDriverStrength);}
        else
            {rumbleCodriver.setRumble(RumbleType.kLeftRumble,0);}
        // put queue contents to dashboard, for debugging / verification.
        SmartDashboard.putString("DriverRight Rumble Queue",DR_Request.toString());
        SmartDashboard.putString("CoDriverLeft Rumbler Queue",CL_Request.toString());
        SmartDashboard.putString("DriverLeft Rumble Queue",DL_Request.toString());
        SmartDashboard.putString("CoDriverRight Rumble Queue",CR_Request.toString());
    }
}
