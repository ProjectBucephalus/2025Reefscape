package frc.robot.util.controlTransmutation;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;

/** Utility object for handling multiple FieldObjects simultaneously */
public class ObjectList extends FieldObject
{
  private ArrayList<FieldObject> fieldObjects;
  private Translation2d controlOutput;
  private boolean active;

  public ObjectList()
  {
    fieldObjects = new ArrayList<FieldObject>();
    active = true;
  }

  public Translation2d process(Translation2d controlInput)
  {
    if (!active || fieldObjects.size() == 0)
      {return controlInput;}
    
    fetchRobotPos();
    controlOutput = controlInput;

    for (int i = fieldObjects.size() - 1; i >= 0; i--)
    {
      controlOutput = fieldObjects.get(i).process(controlOutput);
    }

    return controlOutput;
  }

  /**
   * Adds the given object to the end of the list
   * @param newObject any new FieldObject to be added
   * @return this object list with the new item
   */
  public ObjectList add(FieldObject newObject)
  {
    fieldObjects.add(newObject);
    return this;
  }

  /**
   * Adds the given object to the start of the list so it won't be subject to edge-case conflicts </p>
   * Primarily intended for the outer wall
   * @param newObject any new FieldObject to be added as a high-priority
   * @return this object list with the new item
   */
  public ObjectList addPriority(FieldObject newObject)
  {
    fieldObjects.add(0, newObject);
    return this;
  }

  /** Enables this object list for processing */
  public void setActive()
    {active = true;}
  
  /** Disables this object list for processing */
  public void setInactive()
    {active = false;}
}