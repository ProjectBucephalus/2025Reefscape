package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
    private final CommandXboxController controlXbox = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int brakeAxis = XboxController.Axis.kRightTrigger.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private Limelight s_Limelight = new Limelight();
    private ControllerLayering s_ControllerLayering = new ControllerLayering();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() 
    {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> driver.getRawAxis(brakeAxis),
                () -> false,
                () -> !driver.leftTrigger().getAsBoolean()
            )
        );

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

        /* Command Layering */
        controlXbox.x().and(controlXbox.rightTrigger(0.2))
            .whileTrue(new InstantCommand(() -> s_ControllerLayering.f1(true))).onFalse(new InstantCommand(() -> s_ControllerLayering.f1(false)));
        
        controlXbox.x().and(controlXbox.leftTrigger(0.2))
            .whileTrue(new InstantCommand(() -> s_ControllerLayering.f2(true))).onFalse(new InstantCommand(() -> s_ControllerLayering.f2(false)));

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
