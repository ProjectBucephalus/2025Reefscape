package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeStatus;

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
    private final Swerve s_Swerve = new Swerve(Constants.Swerve.initialHeading);
    private Limelight s_Limelight = new Limelight(s_Swerve);

    private final Intake s_Intake = new Intake();

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

        //s_Swerve.gyro.setYaw(Constants.Swerve.initialHeading);

        // Configure the button bindings
        configureButtonBindings();
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
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driver.back().onTrue(new InstantCommand(() -> s_Swerve.zeroPose()));

        driver.leftTrigger().whileTrue(new InstantCommand(() -> s_Intake.setIntakeStatus(IntakeStatus.INTAKE_CORAL)));
        driver.leftBumper().whileTrue(new InstantCommand(() -> s_Intake.setIntakeStatus(IntakeStatus.INTAKE_ALGAE)));

        /*!TODO drop piece */
        driver.rightBumper().whileTrue(new Test("dropPiece", "dropPiece"));

        /* driveToCage */
        driver.y().and(driver.povUp()).onTrue(new Test("autoDrive", "drive centre cage"));
        driver.y().and(driver.povLeft()).onTrue(new Test("autoDrive", "drive left cage"));
        driver.y().and(driver.povRight()).onTrue(new Test("autoDrive", "drive right cage"));

        /* driveToStationCentre */
        driver.x().and(driver.povUp()).onTrue(new Test("autoDrive", "drive centre station"));
        driver.x().and(driver.povLeft()).onTrue(new Test("autoDrive", "drive left station"));
        driver.x().and(driver.povRight()).onTrue(new Test("autoDrive", "drive right station"));

        /* driveToProcessorCentre */
        driver.b().and(driver.povUp()).onTrue(new Test("autoDrive", "drive centre processor"));
        driver.b().and(driver.povLeft()).onTrue(new Test("autoDrive", "drive left processor"));
        driver.b().and(driver.povRight()).onTrue(new Test("autoDrive", "drive right processor"));

        /* driveToReefCentre */
        driver.a().and(driver.povUp()).onTrue(new Test("autoDrive", "drive centre reef"));
        driver.a().and(driver.povLeft()).onTrue(new Test("autoDrive", "drive left reef"));
        driver.a().and(driver.povRight()).onTrue(new Test("autoDrive", "drive right reef"));

        driver.rightStick().onTrue(new Test("targetObj", "Target Obj"));


        /* Copilot Buttons*/
        


    }

    public Swerve getSwerve()
    {
        return s_Swerve;
    }

    public Limelight getLimelight()
    {
        return s_Limelight;
    }
}
