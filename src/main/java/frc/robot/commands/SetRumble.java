package frc.robot.commands;

import frc.robot.subsystems.Rumbler;
import frc.robot.subsystems.Rumbler.Sides;
import edu.wpi.first.wpilibj2.command.Command;

public class SetRumble extends Command 
{
    // command framework to call the AddRequest method in the rumbler subsystem.
    
    private Rumbler s_Rumbler;
    private Sides aside;
    private String request;

    public SetRumble(Rumbler s_Rumbler,Sides aside, String requestID)
    {
        this.s_Rumbler = s_Rumbler;
        this.aside = aside;
        request = requestID;
    }

    @Override
    public void execute()
    {
        s_Rumbler.addRequest(aside,request);
    }

    public boolean isFinished()
    {
        return true;
    }
}
