package frc.robot.subsystems;

import frc.robot.util.GeoFenceObject;
import frc.robot.util.Conversions;
import frc.robot.util.SwerveModule;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public static SwerveDriveOdometry swerveOdometry;
    public static SwerveModule[] mSwerveMods;
    public static Pigeon2 gyro;

    private static GeoFenceObject[] fieldGeoFence = FieldConstants.GeoFencing.fieldGeoFence;

    private double maxDriveSpeed = Constants.Swerve.maxSpeed;
    private static double maxThrottle = Constants.ControlConstants.maxThrottle;
    private static double minThrottle = Constants.ControlConstants.minThrottle;
    private static double maxRotThrottle = Constants.ControlConstants.maxRotThrottle;
    private static double minRotThrottle = Constants.ControlConstants.minRotThrottle;
    private static double manualRotationScalar = Constants.ControlConstants.manualRotationScalar;
    private static double maxRotationSpeed = Constants.ControlConstants.maxRotationSpeed;
    private static double targetAngle = 0;
    private static double robotRadius = FieldConstants.GeoFencing.robotRadius;
    private boolean manualAngleFlag = false;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
    }

    // ------------------------------------------------------------------------------------------ //
    // | # 5985 Additional drive functions to provide more customisable driving functionality # | //
    // | #                                                                                    # | //

    /**
     * Converts assorted inputs into a tuneable drive profile
     * @param translationVal [-1..1] forward drive axis
     * @param strafeVal [-1..1] sideways drive axis
     * @param targetDelta [-1..1] rotation axis, changes the target angle for the robot to face
     * @param brakeVal [0..1] brake/accelerate axis, modifies the stick input between max and min speeds
     * @param fieldRelative BOOLEAN drive relative to field, false to drive relative to robot front
     * @param fenced BOOLEAN keep robot within defined geofence
     * @author 5985
     */
    public void drive(double translationVal, double strafeVal, double targetDelta, double brakeVal, boolean fieldRelative)
    {
        Translation2d stickInput = new Translation2d(translationVal, strafeVal);

        if (targetDelta != 0 && !manualAngleFlag)
        {
            manualAngleFlag = true;
        }

        targetAngle = MathUtil.inputModulus(targetAngle, -180, 180); // Wraps value [-180..180]
        double targetOffset = targetAngle - getHeading().getDegrees(); // Difference between current and target angles
        
        /* Optimises targetOffset direction (if direct turn would go further than 180 degrees, go opposite direction) */
        if (targetOffset > 180)
        { targetOffset -= 360; }
        else if (targetOffset < -180)
        { targetOffset += 360; }
        
        /* 
         *  Triggers the first cycle after manual input ends 
         *  Reduces target offset and target angle to reduce overswing  
         */
        if (targetDelta == 0 && manualAngleFlag)
        {
            manualAngleFlag = false;
            targetOffset = targetOffset / Constants.ControlConstants.overswingReduction;
            targetAngle = targetAngle - targetOffset;
        }
        /* Changes target angle based on scaled joystick position */
        else
        {
            targetAngle = getHeading().getDegrees() + targetDelta * manualRotationScalar;
        }

        /* Apply deadband to target offset to prevent jittering */
        if (Math.abs(targetOffset) <= Constants.ControlConstants.stickDeadband)
            {targetOffset = 0;}

        /* Calculates rotation value based on target offset and max speed */
        double rotationVal = Conversions.clamp(Math.toRadians(targetOffset) * maxRotationSpeed);
        
        /** Checks if brakes are at all pressed; if not, skips calculations */
        if(brakeVal != 0)
        {       
            /* 
            *  Identical calculations for rotation and translation, different values
            *  Calculates brake range and multiplies by brake value, then inverts to get proper speed scalar 
            *  Multiples input speed by calculated speed scalar
            */
            stickInput = stickInput.times(maxThrottle - ((maxThrottle - minThrottle) * brakeVal));
            rotationVal *= (maxRotThrottle - ((maxRotThrottle - minRotThrottle) * brakeVal));
        }
        else
        {   
            /* If no brakes applied, scales input speed by maximum speed */
            stickInput = stickInput.times(maxThrottle);
        }

        drive
        (
            stickInput, 
            rotationVal,
            fieldRelative, 
            true
        );
    }    

    /**
     * Converts assorted inputs into a tuneable drive profile
     * @param translationVal [-1..1] forward drive axis
     * @param strafeVal [-1..1] sideways drive axis
     * @param targetDelta [-1..1] rotation axis, changes the target angle for the robot to face
     * @param brakeVal [0..1] brake/accelerate axis, modifies the stick input between max and min speeds
     * @param fieldRelative BOOLEAN drive relative to field, false to drive relative to robot front
     * @param fenced BOOLEAN keep robot within defined geofence
     * @author 5985
     */
    public void drive(double translationVal, double strafeVal, double targetDelta, double brakeVal)
    {
        drive(translationVal, strafeVal, targetDelta, brakeVal, true);
    }

    /**
     * Converts assorted inputs into a tuneable drive profile
     * @param translationVal [-1..1] forward drive axis
     * @param strafeVal [-1..1] sideways drive axis
     * @param targetDelta [-1..1] rotation axis, changes the target angle for the robot to face
     * @param brakeVal [0..1] brake/accelerate axis, modifies the stick input between max and min speeds
     * @param fieldRelative BOOLEAN drive relative to field, false to drive relative to robot front
     * @param fenced BOOLEAN keep robot within defined geofence
     * @author 5985
     */
    public void driveFenced(double translationVal, double strafeVal, double targetDelta, double brakeVal, boolean fenced)
    {
        Translation2d motionXY = new Translation2d(translationVal,strafeVal);
        if (fenced)
        {
            // Read down the list of geofence objects
            // Outer wall is index 0, so has highest authority by being processed last
            for (int i = fieldGeoFence.length - 1; i >= 0; i--)
            {
                Translation2d inputDamping = fieldGeoFence[i].dampMotion(getPose().getTranslation(), motionXY, robotRadius);
                motionXY = inputDamping;
            }
        }
        drive(motionXY.getX(), motionXY.getY(), targetDelta, brakeVal, true);
    }

    // | #                                                                                    # | //
    // | # 5985 Additional drive functions to provide more customisable driving functionality # | //
    // ------------------------------------------------------------------------------------------ //


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
    {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxDriveSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) 
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxDriveSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public static SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public static Pose2d getPose() 
    {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) 
    {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading()
    {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading)
    {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public static void zeroHeading()
    {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public static Rotation2d getGyroYaw() 
    {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute()
    {
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setMaxThrottle(double newMaxSpeed)
        {maxThrottle = newMaxSpeed;}

    public void setMinThrottle(double newMinSpeed)
        {minThrottle = newMinSpeed;}

    public void setMaxRotThrottle(double newMaxSpeed)
        {maxRotThrottle = newMaxSpeed;}

    public void setMinRotThrottle(double newMinSpeed)
        {minRotThrottle = newMinSpeed;}

    public void setManualRotationScalar(double newRotationSpeed)
        {manualRotationScalar = newRotationSpeed;}

    public double getTarget()
        {return targetAngle;}

    public void setTarget(double newTargetAngle)
        {targetAngle = newTargetAngle;}

    /** Zero robot headding and reset target angle */
    public void zeroHeading(double targetAngle)
    {
        Swerve.zeroHeading();
        setTarget(0);
    }
    
    /** Zero robot position */
    public void zeroPose(double targetAngle)
    {
        setPose(new Pose2d(new Translation2d(), getHeading()));
    }

    public void shiftPose(double xShift, double yShift)
    {
        setPose(new Pose2d(new Translation2d(getPose().getX() - xShift, getPose().getY() - yShift), getHeading()));
    }

    public void setPoseX(double xPos)
    {
        setPose(new Pose2d(new Translation2d(xPos, getPose().getY()), getHeading()));
    }

    public void setPoseY(double yPos)
    {
        setPose(new Pose2d(new Translation2d(getPose().getX(), yPos), getHeading()));
    }

    public void setPoseXY(double xPos, double yPos)
    {
        setPose(new Pose2d(new Translation2d(xPos, yPos), getHeading()));
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        SmartDashboard.putNumber("X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Y", getPose().getTranslation().getY());

        /* for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        } */
    }
}