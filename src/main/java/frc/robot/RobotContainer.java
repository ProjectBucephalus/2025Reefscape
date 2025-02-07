package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeStatus;
import frc.robot.util.DynamicAuto;
import frc.robot.util.LightLayer;
import frc.robot.util.LightLayer.LEDType;
import frc.robot.util.LightLayer.Mode;

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
    private final Limelight s_Limelight = new Limelight(s_Swerve);
    public static final Diffector s_Diffector = new Diffector();
    private final Climber s_Climber = new Climber();
    private final Intake s_Intake = new Intake();
    private final Rumbler s_Rumbler = new Rumbler(driver, copilot);
    private final LEDRenderer s_Lights = new LEDRenderer();
    private LightLayer progressLayer = new LightLayer(s_Swerve, "Progress");
    private LightLayer statusLayer = new LightLayer(s_Swerve, "Status");
    private LightLayer reefPointerLayer = new LightLayer(s_Swerve, "ReefPointer");
    private LightLayer processorPointerLayer = new LightLayer(s_Swerve, "ProcPointer");

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

        progressLayer.setBorder(true);
        progressLayer.setMode(Mode.DRIVERFACE);
        progressLayer.setType(LEDType.PROGRESS);
        progressLayer.setPriority(9);
        progressLayer.setBorderColor(Color.kBlueViolet);
        progressLayer.setProgress(0.5);
        progressLayer.setWidth(30);

        statusLayer.setMode(Mode.TARGETFACE);
        statusLayer.setType(LEDType.STATUS);
        statusLayer.setPriority(8);
        statusLayer.setStatus(0, true);
        statusLayer.setStatus(2, true);
        statusLayer.setBorder(true);
        statusLayer.setTarget(new Translation2d(1.0,FieldConstants.fieldWidth));

        reefPointerLayer.setMode(Mode.TARGETFACE);
        reefPointerLayer.setType(LEDType.POINTER);
        reefPointerLayer.setWidth(3);
        reefPointerLayer.setBorder(false);
        reefPointerLayer.setColor(Color.kPurple, Color.kBlack);
        reefPointerLayer.setPriority(4);
        reefPointerLayer.setTarget(new Translation2d(4.5,4));

        processorPointerLayer.setMode(Mode.TARGETFACE);
        processorPointerLayer.setType(LEDType.POINTER);
        processorPointerLayer.setColor(Color.kCoral, Color.kBlack);
        processorPointerLayer.setWidth(7);
        processorPointerLayer.setBorder(false);
        processorPointerLayer.setPriority(3);
        processorPointerLayer.setTarget(FieldConstants.driverRed1);

        s_Lights.addLayer(progressLayer);
        s_Lights.addLayer(statusLayer);
        s_Lights.addLayer(reefPointerLayer);
        s_Lights.addLayer(processorPointerLayer);

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

        driver.leftTrigger().whileTrue(Commands.runOnce(() -> s_Intake.setIntakeStatus(IntakeStatus.INTAKE_CORAL)));
        driver.leftBumper().whileTrue(Commands.runOnce(() -> s_Intake.setIntakeStatus(IntakeStatus.INTAKE_ALGAE)));

        /*!TODO drop piece */
        driver.rightBumper().whileTrue(new Test("dropPiece", "dropPiece"));

        /* driveToCage */
        driver.y().and(driver.povUp()).whileTrue(new Test("autoDrive", "drive centre cage"));
        driver.y().and(driver.povLeft()).whileTrue(new Test("autoDrive", "drive left cage"));
        driver.y().and(driver.povRight()).whileTrue(new Test("autoDrive", "drive right cage"));

        /* driveToStationCentre */
        driver.x().and(driver.povUp()).whileTrue(new Test("autoDrive", "drive centre station"));
        driver.x().and(driver.povLeft()).whileTrue(new Test("autoDrive", "drive left station"));
        driver.x().and(driver.povRight()).whileTrue(new Test("autoDrive", "drive right station"));

        /* driveToProcessorCentre */
        driver.b().and(driver.povUp()).onTrue(new Test("autoDrive", "drive centre processor"));
        driver.b().and(driver.povLeft()).onTrue(new Test("autoDrive", "drive left processor"));
        driver.b().and(driver.povRight()).onTrue(new Test("autoDrive", "drive right processor"));

        /* driveToReefCentre */
        driver.a().and(driver.povUp()).whileTrue(new Test("autoDrive", "drive centre reef"));
        driver.a().and(driver.povLeft()).whileTrue(new Test("autoDrive", "drive left reef"));
        driver.a().and(driver.povRight()).whileTrue(new Test("autoDrive", "drive right reef"));

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

        /* Copilot Buttons*/
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
