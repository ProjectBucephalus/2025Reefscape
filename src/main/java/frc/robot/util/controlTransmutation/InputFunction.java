package frc.robot.util.controlTransmutation;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.util.Conversions; 

/** Add your docs here. */
public abstract class InputFunction implements InputTransmuter
{  
  public class Joystick
  {
    private InputCurve inputCurve = new InputCurve();
    private Deadband deadband = new Deadband();
    private Brake brake = null;

    public Translation2d process(Translation2d controlInput)
    {
      return inputCurve.process(deadband.process(controlInput)).times(brake != null ? brake.get() : 1);
    }

    public Joystick withInputCurve(InputCurve inputCurve)
    {
      this.inputCurve = inputCurve;
      return this;
    }

    public Joystick withDeadband(Deadband deadband)
    {
      this.deadband = deadband;
      return this;
    }

    public Joystick withBrake(Brake brake)
    {
      this.brake = brake;
      return this;
    }
  }

  public class InputCurve
  {
    private double power;

    /** 
     * Parabolic curve on axis input, 
     * @param power optional, default 1 for linear
     */
    public InputCurve()
      {this(1);}
    
    /** 
     * Parabolic curve on axis input, 
     * @param power optional, default 1 for linear
     */
    public InputCurve(double power)
      {this.power = power;}

    public Translation2d process(Translation2d controlInput)
    {
      return Conversions.clamp
      (
        new Translation2d
        (
          Math.pow(controlInput.getX(), power), 
          Math.pow(controlInput.getY(), power)
        )
      );
    }
  }

  public class Brake
  {
    private DoubleSupplier brakeAxis;
    private double max;
    private double min;

    public Brake(DoubleSupplier brakeAxis, double maxThrottle, double minThrottle)
    {
      this.brakeAxis = brakeAxis;
      max = maxThrottle;
      min = minThrottle;
    }

    public double get()
    {
      return MathUtil.interpolate(min, max, brakeAxis.getAsDouble());
    }
  }

  public class Deadband
  {
    protected double deadband;

    public Deadband()
      {this(Constants.Control.stickDeadband);}
    
    public Deadband(double deadband)
      {this.deadband = deadband;}
    
    public Translation2d process(Translation2d controlInput)
    {
      return (controlInput.getNorm() <= deadband ? Translation2d.kZero : controlInput);
    }
  }

  public class CrossDeadband extends Deadband
  {
    protected double separation;
    
    public CrossDeadband()
    {
      this(Constants.Control.stickDeadband, 1);
    }

    /**
     * Snaps the input to be purely cardinal
     * @param deadband Size of centre deadband
     * @param separation Determines the size and behaviour of corners: <1 deadzone, >1 smooth control 
     * @return 
     */
    public CrossDeadband(double deadband, double separation)
    {
      super.deadband = deadband;
      this.separation = separation;
    }

    public Translation2d process(Translation2d controlInput)
    {
      if (controlInput.getNorm() <= deadband)
        {return Translation2d.kZero;}
        
      return new Translation2d
      (
        Math.abs(controlInput.getX()) < separation * Math.abs(controlInput.getY()) ? 0 : controlInput.getX(),
        Math.abs(controlInput.getY()) < separation * Math.abs(controlInput.getX()) ? 0 : controlInput.getY()  
      );
    }
  }
}
