package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.commands.AlgaeManipulator.*;
import frc.robot.commands.Auto.*;
import frc.robot.commands.Auto.PathfindToReef.DpadOptions;
import frc.robot.commands.CoralManipulator.*;
import frc.robot.commands.Diffector.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Rumble.*;
import frc.robot.commands.Util.*;
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeStatus;
import frc.robot.subsystems.Rumbler.Sides;
import frc.robot.util.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
    public enum HeadingStates{UNLOCKED, REEF_LOCK, PROCESSOR_LOCK, STATION_LOCK, CAGE_LOCK}

    public static HeadingStates headingState = HeadingStates.UNLOCKED;

    public static boolean coral;
    public static boolean algae;

    /* Controllers */
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController copilot = new CommandXboxController(1);

    /* Subsystems */
    public static final CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
    public static final Limelight s_Limelight = new Limelight(s_Swerve);
    public static final Diffector s_Diffector = new Diffector();
    public static final Climber s_Climber = new Climber();
    public static final Intake s_Intake = new Intake();
    public static final CoralManipulator s_CoralManipulator = new CoralManipulator();
    public static final AlgaeManipulator s_AlgaeManipulator = new AlgaeManipulator();
    public static Rumbler s_Rumbler = new Rumbler(driver, copilot);

    /* Drive Controls */
    public static final int translationAxis = XboxController.Axis.kLeftY.value;
    public static final int strafeAxis = XboxController.Axis.kLeftX.value;
    public static final int rotationAxis = XboxController.Axis.kRightX.value;
    public static final int brakeAxis = XboxController.Axis.kRightTrigger.value;

    /* Triggers */
    public static final Trigger unlockHeadingTrigger = new Trigger(() -> Math.abs(driver.getRawAxis(rotationAxis)) > Constants.Control.stickDeadband);
    private final Trigger cageDriveTrigger = new Trigger(() -> headingState == HeadingStates.CAGE_LOCK);
    private final Trigger reefDriveTrigger = new Trigger(() -> headingState == HeadingStates.REEF_LOCK);
    private final Trigger stationDriveTrigger = new Trigger(() -> headingState == HeadingStates.STATION_LOCK);
    private final Trigger processorDriveTrigger = new Trigger(() -> headingState == HeadingStates.PROCESSOR_LOCK);
    private final Trigger driverLeftRumbleTrigger = new Trigger(() -> s_Intake.getCoralSwitch1State() || s_Intake.getCoralSwitch2State() || !s_Intake.getAlgaeBeamBreakState() );
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
        s_Swerve.setDefaultCommand
        (
            new TeleopSwerve
            (
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> driver.getRawAxis(brakeAxis),
                () -> true,
                () -> !driver.leftStick().getAsBoolean()
            )
        );

        s_Swerve.resetRotation(new Rotation2d(Units.degreesToRadians(Constants.Swerve.initialHeading)));

        // Configure the button bindings
        configureButtonBindings();
    
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putString("Auto Input", Constants.Auto.defaultAuto);
        SmartDashboard.putData
        (
            "Swerve Drive", 
            new Sendable() 
            {
                @Override
                public void initSendable(SendableBuilder builder) 
                {
                    builder.setSmartDashboardType("SwerveDrive");

                    builder.addDoubleProperty("Front Left Angle", () -> s_Swerve.getModule(0).getCurrentState().angle.getRadians(), null);
                    builder.addDoubleProperty("Front Left Velocity", () -> s_Swerve.getModule(0).getCurrentState().speedMetersPerSecond, null);

                    builder.addDoubleProperty("Front Right Angle", () -> s_Swerve.getModule(1).getCurrentState().angle.getRadians(), null);
                    builder.addDoubleProperty("Front Right Velocity", () -> s_Swerve.getModule(1).getCurrentState().speedMetersPerSecond, null);

                    builder.addDoubleProperty("Back Left Angle", () ->s_Swerve.getModule(2).getCurrentState().angle.getRadians(), null);
                    builder.addDoubleProperty("Back Left Velocity", () ->s_Swerve.getModule(2).getCurrentState().speedMetersPerSecond, null);

                    builder.addDoubleProperty("Back Right Angle", () -> s_Swerve.getModule(3).getCurrentState().angle.getRadians(), null);
                    builder.addDoubleProperty("Back Right Velocity", () -> s_Swerve.getModule(3).getCurrentState().speedMetersPerSecond, null);

                    builder.addDoubleProperty("Robot Angle", () -> s_Swerve.getState().Pose.getRotation().getRadians(), null);
                }
            }
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() 
    {
        /* Driver Buttons */
        driver.start().onTrue(Commands.runOnce(() -> s_Swerve.seedFieldCentric()));
        driver.back().onTrue(new AutoScoreSequence(s_Diffector, s_AlgaeManipulator, s_CoralManipulator, s_Swerve, () -> s_Swerve.getState().Pose.getTranslation()));

        driver.leftTrigger().whileTrue(Commands.runOnce(() -> s_Intake.setIntakeStatus(IntakeStatus.INTAKE_CORAL)));
        driver.leftBumper().whileTrue(Commands.runOnce(() -> s_Intake.setIntakeStatus(IntakeStatus.INTAKE_ALGAE)));

        driver.rightBumper().whileTrue(new DropGamePiece(s_AlgaeManipulator, s_CoralManipulator));

        //driver.rightStick().whileTrue(new Test("targetObj", "Target Obj")); Leave commented until we decide on object tracking

        /* Binds the buttons and triggers that set the heading state */
        unlockHeadingTrigger.onTrue(Commands.runOnce(() -> headingState = HeadingStates.UNLOCKED));
        driver.y().onTrue(Commands.runOnce(() -> headingState = HeadingStates.CAGE_LOCK));
        driver.a().onTrue(Commands.runOnce(() -> headingState = HeadingStates.STATION_LOCK));
        driver.b().onTrue(Commands.runOnce(() -> headingState = HeadingStates.PROCESSOR_LOCK));
        driver.x().onTrue(Commands.runOnce(() -> headingState = HeadingStates.REEF_LOCK));

        /* Binds the Pathfinding command to run for the cage when the cage trigger is active and a corresponding dpad direction is pressed */
        cageDriveTrigger.and(driver.povUp()).onTrue(new PathfindToAndFollow("cage2", s_Swerve));
        cageDriveTrigger.and(driver.povLeft()).onTrue(new PathfindToAndFollow("cage3", s_Swerve));
        cageDriveTrigger.and(driver.povRight()).onTrue(new PathfindToAndFollow("cage1", s_Swerve));

        /* Binds the Coral Station Pathfinding command to run when the station trigger is active and a corresponding dpad direction is pressed */
        stationDriveTrigger.and(driver.povUp()).onTrue(new PathfindToStation(5, () -> s_Swerve.getState().Pose.getY(), s_Swerve));
        stationDriveTrigger.and(driver.povLeft()).onTrue(new PathfindToStation(2, () -> s_Swerve.getState().Pose.getY(), s_Swerve));
        stationDriveTrigger.and(driver.povRight()).onTrue(new PathfindToStation(8, () -> s_Swerve.getState().Pose.getY(), s_Swerve));

        /* Binds the Pathfinding command to run for the processor when the processor trigger is active and right is pressed on the dpad */
        processorDriveTrigger.and(driver.povRight()).onTrue(new PathfindToAndFollow("p", s_Swerve));

        /* Binds the Reef Pathfinding command to run when the reef trigger is active and a corresponding dpad direction is pressed */
        reefDriveTrigger.and(driver.povUp()).onTrue(new PathfindToReef(DpadOptions.CENTRE, () -> s_Swerve.getState().Pose.getTranslation(), s_Swerve));
        reefDriveTrigger.and(driver.povLeft()).onTrue(new PathfindToReef(DpadOptions.LEFT, () -> s_Swerve.getState().Pose.getTranslation(), s_Swerve));
        reefDriveTrigger.and(driver.povRight()).onTrue(new PathfindToReef(DpadOptions.RIGHT, () -> s_Swerve.getState().Pose.getTranslation(), s_Swerve));


        driverLeftRumbleTrigger.onTrue(new SetRumble(s_Rumbler, Sides.DRIVER_LEFT, "Intake Full"));
        /* 
         * Binds heading targetting commands to run while the appropriate trigger is active and the dpad isn't pressed
         * Does not need to check the rotation stick, as soon at the rotation stick is moved all drive triggers become false (see line 141)
         */
        cageDriveTrigger.and(driver.povCenter())
            .whileTrue
            (
                new TargetHeading
                (
                    s_Swerve, 
                    Rotation2d.kZero,
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> driver.getRawAxis(brakeAxis),
                    () -> !driver.leftStick().getAsBoolean()
                )
            );

        stationDriveTrigger.and(driver.povCenter())
            .whileTrue
            (
                new TargetHeadingStation
                (
                    s_Swerve, 
                    () -> s_Swerve.getState().Pose.getY(),
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> driver.getRawAxis(brakeAxis),
                    () -> !driver.leftStick().getAsBoolean()
                )
            );
        
        processorDriveTrigger.and(driver.povCenter())
            .whileTrue
            (
                new TargetHeading
                (
                    s_Swerve, 
                    Rotation2d.kCW_90deg,
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> driver.getRawAxis(brakeAxis),
                    () -> !driver.leftStick().getAsBoolean()
                )
            );

        reefDriveTrigger.and(driver.povCenter())
            .whileTrue
            (
                new TargetHeadingReef
                (
                    s_Swerve, 
                    () -> s_Swerve.getState().Pose.getTranslation(),
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> driver.getRawAxis(brakeAxis),
                    () -> !driver.leftStick().getAsBoolean()
                )
            );

        /* Copilot Buttons */
        copilot.start().and(copilot.back()).onTrue(new GoToClimbConfig(s_Diffector));
        copilot.back().onTrue(new Test("climber", "deploy"));

        /* scoringCoral */
        copilot.a().and(copilot.rightTrigger().negate()).onTrue(new ScoreCoralSequence(1, s_Diffector, s_CoralManipulator));
        copilot.b().and(copilot.rightTrigger().negate()).onTrue(new ScoreCoralSequence(2, s_Diffector, s_CoralManipulator));
        copilot.x().and(copilot.rightTrigger().negate()).onTrue(new ScoreCoralSequence(3, s_Diffector, s_CoralManipulator));
        copilot.y().and(copilot.rightTrigger().negate()).onTrue(new ScoreCoralSequence(4, s_Diffector, s_CoralManipulator));
        /* scoringAlgae */
        copilot.a().and(copilot.rightTrigger()).onTrue(new ScoreAlgae(false, s_Diffector, s_AlgaeManipulator));
        copilot.b().and(copilot.rightTrigger()).onTrue(new IntakeAlgaeSequence(true, s_Diffector, s_AlgaeManipulator));
        copilot.x().and(copilot.rightTrigger()).onTrue(new IntakeAlgaeSequence(false, s_Diffector, s_AlgaeManipulator));
        copilot.y().and(copilot.rightTrigger()).onTrue(new ScoreAlgae(true, s_Diffector, s_AlgaeManipulator));

        /* transferPosition */
        copilot.povUp().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.TRANSFER_ALGAE));
        copilot.povUp().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new GoToAlgaeHandoverConfig(s_Diffector));
        copilot.povUp().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.TRANSFER_ALGAE));
        copilot.povUp().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new GoToCoralHandoverConfig(s_Diffector));

        /* deploy/TransferGamePieces */
        copilot.povDown().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new IntakeAndStow(6, s_Intake));
        copilot.povDown().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new TransferGamePiece(s_Diffector, s_Intake, false));
        copilot.povDown().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new IntakeAndStow(6, s_Intake));
        copilot.povDown().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new TransferGamePiece(s_Diffector, s_Intake, true));

        /* stowGamePieces */
        copilot.povRight().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new IntakeAndStow(5, s_Intake));
        copilot.povRight().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new IntakeAlgae(s_AlgaeManipulator));
        copilot.povRight().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new IntakeAndStow( 5, s_Intake));
        copilot.povRight().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new IntakeCoral(s_CoralManipulator));

        /* intakeFromCoralStation */
        copilot.povLeft().and(copilot.rightTrigger().negate()).whileTrue(new IntakeCoralSequence(s_Diffector, s_CoralManipulator));

        /* arm/IntakeGrab */
        copilot.leftTrigger().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new IntakeAndStow( 1, s_Intake));
        copilot.leftTrigger().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new IntakeAlgae(s_AlgaeManipulator));
        copilot.leftTrigger().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new IntakeAndStow( 2, s_Intake));
        copilot.leftTrigger().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new IntakeCoral(s_CoralManipulator));

        /* arm/IntakeRelease */
        copilot.leftBumper().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new IntakeAndStow(3, s_Intake));
        copilot.leftBumper().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new EjectAlgae(s_AlgaeManipulator));
        copilot.leftBumper().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new IntakeAndStow(4, s_Intake));
        copilot.leftBumper().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new EjectCoral(s_CoralManipulator));

        /* robotModifiers */
        copilot.rightTrigger().onTrue(new Test("algaeModifier", "on")).onFalse(new Test("algaeModifier", "off"));
        copilot.rightBumper().onTrue(new Test("intakeModifier", "on")).onFalse(new Test("intakeModifier", "off"));

        // copilot.leftStick().and(copilot.rightTrigger()).onTrue(new Test("manualIntake/Winch", null));
        // copilot.rightStick().and(copilot.rightTrigger()).onTrue(new Test("manualDiffector", null));
    }

    public CommandSwerveDrivetrain getSwerve()
    {
        return s_Swerve;
    }

    public Limelight getLimelight()
    {
        return s_Limelight;
    }

    public Command getAutoCommand()
    {
        // Gets the input string of command phrases, processes into a list of commands, and puts them into a sequential command group
        return new RunAutoCommandList(DynamicAuto.getCommandList(SmartDashboard.getString("Auto Input", Constants.Auto.defaultAuto)));
    }
}
