package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MechanismConstants 
{
  public static class ClimberConfigs
  {
    public static final double winchBalanceScalar = 0.05;

    public static final double winchPlanetaryRatio = 75;
    public static final double winchGearIn   = 20;
    public static final double winchGearOut  = 60;
    public static final double winchChainIn  = 12;
    public static final double winchChainOut = 24;
    public static final double winchGearRatio = ((winchGearOut / winchGearIn) * (winchChainOut / winchChainIn) * winchPlanetaryRatio);
    public static final double winchDefaultCruise = 1;
    public static final double winchClimbCruise = 0.5;
    public static final double winchMotionMagicAccel  = 1;

    public static final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
    static
    {
      climberMotorConfig.Feedback.SensorToMechanismRatio = winchGearRatio;
      climberMotorConfig.MotionMagic.MotionMagicCruiseVelocity = winchDefaultCruise;
      climberMotorConfig.MotionMagic.MotionMagicAcceleration = winchMotionMagicAccel;
      climberMotorConfig.Slot0.kP = 150;
      climberMotorConfig.Slot0.kI = 0;
      climberMotorConfig.Slot0.kD = 0;
      climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      climberMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    };
  }
  
  public static class DiffectorConfigs
  {    
    public static final double motorStallCurrent = 80; // TODO: Tune this to the point that it will reliably prevent stalls

    private static final double diffectorGearTeethIn = 8;
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
    public static final double diffectorAlgaeCruiseMotor = 75;
    /** Desired acceleration of Motor for Elevation, RPS^2 */
    public static final double diffectorElevationAccelerationMotor = 70;
    /** Desired acceleration of Motor for Rotation, RPS^2 */
    public static final double diffectorRotationAccelerationMotor = 70;
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
    
    public static final TalonFXConfiguration diffectorMotorConfig = new TalonFXConfiguration();
    static
    {
      /* Diffector Motor Gneral Config */
      diffectorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      diffectorMotorConfig.Feedback.SensorToMechanismRatio = gearboxRatio;

      /* Diffector Motor Config (Default) */
      diffectorMotorConfig.Slot0.kG = 0.225;
      diffectorMotorConfig.Slot0.kS = 0.05;
      diffectorMotorConfig.Slot0.kV = 0.58;
      diffectorMotorConfig.Slot0.kP = 100.0;
      diffectorMotorConfig.Slot0.kI = 0.0;
      diffectorMotorConfig.Slot0.kD = 0.0;
      
      /* Diffector Motor Config (Virtual Spring) */
      diffectorMotorConfig.Slot1.kG = 0.0;
      diffectorMotorConfig.Slot1.kS = 0.0;
      diffectorMotorConfig.Slot1.kV = 0.0;
      diffectorMotorConfig.Slot1.kP = 3.0;
      diffectorMotorConfig.Slot1.kI = 0.0;
      diffectorMotorConfig.Slot1.kD = 0.0;

      /* Diffector MotionMagic Default Config */
      diffectorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = diffectorCruise;
      diffectorMotorConfig.MotionMagic.MotionMagicAcceleration = diffectorRotationAcceleration;
    };
  }
}
