package frc.robot.commands;

import frc.robot.subsystems.Rumbler;
import frc.robot.subsystems.Rumbler.Sides;
import edu.wpi.first.wpilibj2.command.Command;

public class CancelRumble extends Command {
    // command framework to call the remove request method in the rumbler subsystem
    private Rumbler s_Rumbler;
    private Sides Aside;
    private String Request;

    public CancelRumble(Rumbler s_Rumbler,Sides Aside, String RequestID){
        this.s_Rumbler = s_Rumbler;
        this.Aside = Aside;
        Request = RequestID;
    }

    @Override
    public void execute(){
        s_Rumbler.removeRequest(Aside,Request);
    }

    public boolean isFinished(){
        return true;
    }
}
