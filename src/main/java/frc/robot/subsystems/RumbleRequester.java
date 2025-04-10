package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RumbleRequester extends SubsystemBase
{
  private ArrayList<String> queue = new ArrayList<String>();
  private final CommandXboxController controller;
  private final RumbleType side;
  private final DoubleSupplier strengthSup;
  private final Consumer<String> outputCns;

  public RumbleRequester(CommandXboxController controller, RumbleType side, Consumer<String> outputCns, DoubleSupplier strengthSup)
  {
    this.controller = controller;
    this.side = side;
    this.outputCns = outputCns;
    this.strengthSup = strengthSup;
  }

  private void addRequest(String requestID)
  {
    if(!queue.contains(requestID))
      {queue.add(requestID);}
  }

  private void removeRequest(String requestID)
    {queue.remove(requestID);}

  public Command requestCommand(boolean addRequest, String requestID)
    {return addRequest ? Commands.runOnce(() -> addRequest(requestID)) : Commands.runOnce(() -> removeRequest(requestID));}

  /** Adds a rumble with the provided ID that runs while the given trigger is true. Returns the subsystem for easier chaining. */
  public RumbleRequester addRumbleTrigger(String requestID, Trigger trigger)
  {
    trigger.whileTrue(Commands.startEnd(() -> addRequest(requestID), () -> removeRequest(requestID)));
    return this;
  }

  public void clearRequests()
    {queue.clear();}

  public Command timedRequestCommand(String requestID, double durationSeconds)
  {
    return 
    Commands.sequence
    (
      requestCommand(true, requestID),
      Commands.waitSeconds(durationSeconds),
      requestCommand(false, requestID)
    );
  }

  @Override
  public void periodic() 
  {
    controller.setRumble(side, queue.isEmpty() ? 0 : strengthSup.getAsDouble());
    outputCns.accept(queue.toString());
  }
}
