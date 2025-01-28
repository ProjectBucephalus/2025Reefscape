package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.Auto.PathfindToAndFollow;
import frc.robot.commands.Auto.PathfindToReef;
import frc.robot.commands.Auto.PathfindToStation;
import frc.robot.commands.Auto.RunAutoCommandList;
import frc.robot.commands.Auto.TargetHeading;
import frc.robot.commands.Auto.PathfindToReef.DpadOptions;
import frc.robot.commands.Intake.SetIntakeStatus;
import frc.robot.commands.Util.Test;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeStatus;
import frc.robot.util.DynamicAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int brakeAxis = XboxController.Axis.kRightTrigger.value;

    /* Subsystems */
    public static final CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
    public static final Limelight s_Limelight = new Limelight(s_Swerve);
    public static final Diffector s_Diffector = new Diffector();
    public static final Climber s_Climber = new Climber();
    public static final Intake s_Intake = new Intake();
    public static final CoralManipulator s_CoralManipulator = new CoralManipulator();
    public static final AlgaeManipulator s_AlgaeManipulator = new AlgaeManipulator();
    public Rumbler s_Rumbler = new Rumbler(driver, copilot);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
        SmartDashboard.putString("Auto Input", Constants.Auto.defaultAuto);

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

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        s_Swerve.resetRotation(new Rotation2d(Math.toRadians(Constants.Swerve.initialHeading)));

        // Configure the button bindings
        configureButtonBindings();

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
        driver.back().onTrue(Commands.runOnce(() -> s_Swerve.tareEverything()));

        driver.leftTrigger().onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.INTAKE_CORAL)).onFalse(new SetIntakeStatus(s_Intake, IntakeStatus.STOWED));
        driver.leftBumper().onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.INTAKE_ALGAE)).onFalse(new SetIntakeStatus(s_Intake, IntakeStatus.STOWED));

        /*!TODO drop piece */
        driver.rightBumper().whileTrue(new Test("dropPiece", "dropPiece"));

        /* driveToCage */
        driver.y().and(driver.povUp()).onTrue(new PathfindToAndFollow("cage2"));
        driver.y().and(driver.povLeft()).onTrue(new PathfindToAndFollow("cage3"));
        driver.y().and(driver.povRight()).onTrue(new PathfindToAndFollow("cage1"));

        /* driveToStation */
        driver.x().and(driver.povUp()).onTrue(new PathfindToStation(5, () -> s_Swerve.getState().Pose.getY()));
        driver.x().and(driver.povLeft()).onTrue(new PathfindToStation(2, () -> s_Swerve.getState().Pose.getY()));
        driver.x().and(driver.povRight()).onTrue(new PathfindToStation(8, () -> s_Swerve.getState().Pose.getY()));

        /* driveToProcessor */
        driver.b().and(driver.povRight()).onTrue(new PathfindToAndFollow("p"));

        /* driveToReef */
        driver.a().and(driver.povUp()).onTrue(new PathfindToReef(DpadOptions.CENTRE, () -> s_Swerve.getState().Pose.getTranslation()));
        driver.a().and(driver.povLeft()).onTrue(new PathfindToReef(DpadOptions.LEFT, () -> s_Swerve.getState().Pose.getTranslation()));
        driver.a().and(driver.povRight()).onTrue(new PathfindToReef(DpadOptions.RIGHT, () -> s_Swerve.getState().Pose.getTranslation()));

        driver.rightStick().whileTrue(new Test("targetObj", "Target Obj"));

        driver.a().and(new Trigger(() -> Math.abs(driver.getRightX()) < Constants.Control.stickDeadband))
            .onTrue
            (
                new TargetHeading
                (
                    s_Swerve, 
                    Rotation2d.kCW_90deg,
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> -driver.getRawAxis(rotationAxis), 
                    () -> driver.getRawAxis(brakeAxis),
                    () -> !driver.leftStick().getAsBoolean()
                )
            );

        driver.y().onTrue(new PathfindToAndFollow("ReefTag"));
        driver.x().onTrue(new PathfindToAndFollow("ReefLeft"));
        driver.b().onTrue(new PathfindToAndFollow("ReefRight"));

        /* Copilot Buttons */
        copilot.start().onTrue(new Test("climber", "activate"));
        copilot.back().onTrue(new Test("climber", "deploy"));


        /* scoringGamePieces */
        copilot.a().and(copilot.rightBumper()).onTrue(new Test("Coral/Algae Scoring", "Lvl 1/Processor"));
        copilot.b().and(copilot.rightBumper()).onTrue(new Test("Coral/Algae Scoring", "Lvl 2"));
        copilot.x().and(copilot.rightBumper()).onTrue(new Test("Coral/Algae Scoring", "Lvl 3"));
        copilot.y().and(copilot.rightBumper()).onTrue(new Test("Coral/Algae Scoring", "Lvl 4/Net"));


        /* transferPosition */
        copilot.povUp().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new Test("transferPosition", null));
        copilot.povUp().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new Test("transferPosition", null));
        copilot.povUp().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new Test("transferPosition", null));
        copilot.povUp().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new Test("transferPosition", "Climbing"));

        /* deploy/TransferGamePieces */
        copilot.povDown().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new Test("deploy/Transfer", null));
        copilot.povDown().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new Test("deploy/Transfer", null));
        copilot.povDown().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new Test("deploy/Transfer", null));
        copilot.povDown().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new Test("deploy/Transfer", null));

        /* stowGamePieces */
        copilot.povRight().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new Test("stowPosition", null));
        copilot.povRight().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new Test("stowPosition", null));
        copilot.povRight().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new Test("stowPosition", null));
        copilot.povRight().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new Test("stowPosition", null));

        /* intakeFromCoralStation */
        copilot.povLeft().and(copilot.rightBumper()).onTrue(new Test("coralStation", null));

        /* arm/IntakeGrab */
        copilot.leftTrigger().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new Test("Arm/Intake", null));
        copilot.leftTrigger().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new Test("Arm/Intake", null));
        copilot.leftTrigger().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new Test("Arm/Intake", null));
        copilot.leftTrigger().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new Test("Arm/Intake", null));

        /* arm/IntakeRelease */
        copilot.leftBumper().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new Test("Arm/Intake", null));
        copilot.leftBumper().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new Test("Arm/Intake", null));
        copilot.leftBumper().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new Test("Arm/Intake", null));
        copilot.leftBumper().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new Test("Arm/Intake", null));

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
