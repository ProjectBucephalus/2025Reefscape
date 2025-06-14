package frc.robot.util.controlTransmutation;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.util.Conversions; 

/** Add your docs here. */
public abstract class InputFunction implements InputTransmuter
{  
  public class Joystick
  {
    private InputCurve inputCurve;
    private Deadband deadband;
    private Brake brake;

    private DoubleSupplier inputX;
    private DoubleSupplier inputY;

    public Joystick(DoubleSupplier inputX, DoubleSupplier inputY)
    {
      this.inputX = inputX;
      this.inputY = inputY;

      inputCurve = new InputCurve();
      deadband = new Deadband();
      brake = null;
    }

    public Translation2d stickOutput()
    {
      return process(new Translation2d(inputX.getAsDouble(), inputY.getAsDouble()));
    }

    public Supplier<Translation2d> stickOutputSup()
    {
      return () -> stickOutput();
    }

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
          Math.copySign(Math.pow(controlInput.getX(), power), controlInput.getX()), 
          Math.copySign(Math.pow(controlInput.getY(), power), controlInput.getY())
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
      return MathUtil.interpolate(max, min, brakeAxis.getAsDouble());
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
    protected double overlap;
    
    public CrossDeadband()
    {
      this(Constants.Control.stickDeadband, 1);
    }

    /**
     * Snaps the input to be purely cardinal
     * @param deadband Size of centre deadband
     * @param overlap Determines the size and behaviour of corners: <1 deadzone, >1 smooth control 
     * @return 
     */
    public CrossDeadband(double deadband, double overlap)
    {
      super.deadband = deadband;
      this.overlap = overlap;
    }

    public Translation2d process(Translation2d controlInput)
    {
      if (controlInput.getNorm() <= deadband)
        {return Translation2d.kZero;}
        
      return new Translation2d
      (
        Math.abs(controlInput.getX()) < overlap * Math.abs(controlInput.getY()) ? 0 : controlInput.getX(),
        Math.abs(controlInput.getY()) < overlap * Math.abs(controlInput.getX()) ? 0 : controlInput.getY()  
      );
    }
  }
}
