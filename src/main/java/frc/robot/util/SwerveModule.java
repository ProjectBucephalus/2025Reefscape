package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.constants.Constants;

public class SwerveModule 
{
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX m_AngleMotor;
    private TalonFX m_DriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        // # 5985 ADDITION >> Pulls previous calibration before it's wiped for persistant calibration!!! # //
        angleEncoder = new CANcoder(moduleConstants.cancoderID); // Create new CANcoder
        CANcoderConfiguration oldConfig = new CANcoderConfiguration(); // Create CANcoder config to pull existing values into
        angleEncoder.getConfigurator().refresh(oldConfig); // Pull previous CANcoder values into config holder
        Robot.ctreConfigs.swerveCANcoderConfig.MagnetSensor.MagnetOffset = oldConfig.MagnetSensor.MagnetOffset; // Extract previous MagnetOffset into standard configs
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig); // Push standard configs with previous MagnetOffset into new CANcoder

        /* Angle Motor Config */
        m_AngleMotor = new TalonFX(moduleConstants.angleMotorID);
        m_AngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        //if(moduleNumber == 1) {mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfigAlt);} // ERROR: The physical gearing on the specific robot is built wrong, remove this if all swerve modules are built correctly
        resetToAbsolute();

        /* Drive Motor Config */
        m_DriveMotor = new TalonFX(moduleConstants.driveMotorID);
        m_DriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        m_DriveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        SwerveModuleState state = desiredState;
        state.optimize(getState().angle); 
        m_AngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            m_DriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            m_DriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder()
    {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute()
    {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        m_AngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
            Conversions.RPSToMPS(m_DriveMotor.getVelocity().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(m_AngleMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(m_DriveMotor.getPosition().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(m_AngleMotor.getPosition().getValueAsDouble())
        );
    }
}