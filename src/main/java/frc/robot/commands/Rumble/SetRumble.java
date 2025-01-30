package frc.robot.commands.Rumble;

import frc.robot.subsystems.Rumbler;
import frc.robot.subsystems.Rumbler.Sides;
import edu.wpi.first.wpilibj2.command.Command;

public class SetRumble extends Command 
{
    // command framework to call the AddRequest method in the rumbler subsystem.
    
    private Rumbler s_Rumbler;
    private Sides side;
    private String request;

    public SetRumble(Rumbler s_Rumbler, Sides side, String requestID)
    {
        this.s_Rumbler = s_Rumbler;
        this.side = side;
        request = requestID;
    }

    @Override
    public void execute()
    {
        s_Rumbler.addRequest(side, request);
    }

    public boolean isFinished()
    {
        return true;
    }
}
