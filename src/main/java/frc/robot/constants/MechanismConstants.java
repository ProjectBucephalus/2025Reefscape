package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MechanismConstants 
{
  public static class AlgaeConfigs
  {
    public static final CurrentLimitsConfigs currentLimits = 
    new CurrentLimitsConfigs()
      .withStatorCurrentLimit(30);
  }

  public static class ClimberConfigs
  {
    public static final double winchBalanceScalar = 1;

    public static final double winchPlanetaryRatio = 45;
    public static final double winchGearIn   = 20;
    public static final double winchGearOut  = 60;
    public static final double winchGearRatio = ((winchGearOut / winchGearIn) * winchPlanetaryRatio);
    public static final double winchDefaultCruise = 100;
    public static final double winchClimbCruise = 100;
    public static final double winchMotionMagicAccel  = 100;

    public static final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
    static
    {
      climberMotorConfig.Feedback.SensorToMechanismRatio = winchGearRatio;
      climberMotorConfig.MotionMagic.MotionMagicCruiseVelocity = winchDefaultCruise;
      climberMotorConfig.MotionMagic.MotionMagicAcceleration = winchMotionMagicAccel;
      climberMotorConfig.Slot0.kP = 100;
      climberMotorConfig.Slot0.kI = 0;
      climberMotorConfig.Slot0.kD = 0;
      climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      climberMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    };
  }
  
  public static class DiffectorConfigs
  {    
    public static final double motorStallCurrent = 160; // TODO: Tune this to the point that it will reliably prevent stalls

    private static final double diffectorGearTeethIn  = 8;
    private static final double diffectorGearTeethOut = 60;
    private static final double diffectorSprocketTeethIn  = 18;
    private static final double diffectorSprocketTeethOut = 72;
    /** Output sprocket degrees per motor rotation */
    public static final double gearboxRatio = (diffectorGearTeethOut / diffectorGearTeethIn);
    /** Ratio of output sprocket to arm sprocket (output sprocket teeth/arm sprocket teeth) */
    public static final double sprocketRatio = (diffectorSprocketTeethIn / diffectorSprocketTeethOut);
    /** Pitch Diameter of the sprocket, in m */
    public static final double sprocketPitchDiameter = 0.036576;
    /** Metres of chain moved per sprocket degree */
    public static final double travelRatio = (sprocketPitchDiameter * Math.PI) / 360;
    /** 
     * Number of arm degrees moved for one motor degree of a single motor 
     * Output sprocket rotations per motor rotation * output sprocket to arm sprocket ratio,
     * divided by 2 to give the contribution of a single motor
     */
    public static final double rotationRatio = (sprocketRatio);

    /** Desired cruise speed of Motor, RPS */
    public static final double diffectorCruiseMotor = 90;
    /** Desired cruise speed of Motor when holding Algae, RPS */
    public static final double diffectorAlgaeCruiseMotor = 60;
    /** Desired acceleration of Motor for Elevation, RPS^2 */
    public static final double diffectorElevationAccelerationMotor = 150;
    /** Desired acceleration of Motor for Rotation, RPS^2 */
    public static final double diffectorRotationAccelerationMotor = 80;
    /** Desired acceleration of Motor for Rotation when holding Algae, RPS^2 */
    public static final double diffectorAlgaeRotationAccelerationMotor = 35;
    
    /** Desired cruise speed of Mechanism, RPS */
    public static final double diffectorCruise = diffectorCruiseMotor / gearboxRatio;
    /** Desired cruise speed of Mechanism when holding Algae, RPS */
    public static final double diffectorAlgaeCruise = diffectorAlgaeCruiseMotor / gearboxRatio;
    /** Desired acceleration of Mechanism for Elevation, RPS^2 */
    public static final double diffectorElevationAcceleration = diffectorElevationAccelerationMotor / gearboxRatio;
    /** Desired acceleration of Mechanism for Rotation, RPS^2 */
    public static final double diffectorRotationAcceleration = diffectorRotationAccelerationMotor / gearboxRatio;
    /** Desired acceleration of Mechanism for Rotation when holding Algae, RPS^2 */
    public static final double diffectorAlgaeRotationAcceleration = diffectorAlgaeRotationAccelerationMotor / gearboxRatio;
    
    public static final TalonFXConfiguration getMotorConfigs()
    {
      TalonFXConfiguration motorConfig = new TalonFXConfiguration();

      /* Diffector Motor Gneral Config */
      motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      motorConfig.Feedback.SensorToMechanismRatio = gearboxRatio;

      /* Diffector Motor Config (Default) */
      motorConfig.Slot0.kG = 0.1755;
      motorConfig.Slot0.kS = 0.1755;
      motorConfig.Slot0.kV = 0.865;
      motorConfig.Slot0.kP = 390;
      motorConfig.Slot0.kI = 0.0;
      motorConfig.Slot0.kD = 0.225;

      /* Diffector MotionMagic Default Config */
      motorConfig.MotionMagic.MotionMagicCruiseVelocity = diffectorCruise;
      motorConfig.MotionMagic.MotionMagicAcceleration = diffectorRotationAcceleration;

      return motorConfig;
    }
  }
}
