package frc.robot.commands.Rumble;

import frc.robot.subsystems.Rumbler;
import frc.robot.subsystems.Rumbler.Sides;
import edu.wpi.first.wpilibj2.command.Command;

public class CancelRumble extends Command 
{
    // command framework to call the remove request method in the rumbler subsystem
    private Rumbler s_Rumbler;
    private Sides side;
    private String Request;

    public CancelRumble(Rumbler s_Rumbler, Sides side, String RequestID)
    {
        this.s_Rumbler = s_Rumbler;
        this.side = side;
        Request = RequestID;
    }

    @Override
    public void execute()
    {
        s_Rumbler.removeRequest(side, Request);
    }

    public boolean isFinished()
    {
        return true;
    }
}
