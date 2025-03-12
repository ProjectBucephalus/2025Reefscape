package frc.robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.Auto.*;
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;
import frc.robot.subsystems.Climber.ClimberStatus;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;
import frc.robot.util.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
  /* Enums */
  public enum HeadingStates{UNLOCKED, REEF_LOCK, PROCESSOR_LOCK, STATION_LOCK, CAGE_LOCK}
  public enum DpadOptions{CENTRE, LEFT, RIGHT}
  
  private final Telemetry logger = new Telemetry(Constants.Swerve.maxSpeed);

  /* Persistent values for tracking systems */
  public static HeadingStates headingState = HeadingStates.UNLOCKED;
  public static boolean coral = Constants.DiffectorConstants.startingCoralState;
  public static boolean algae = Constants.DiffectorConstants.startingAlgaeState;
  public static SwerveDriveState swerveState;

  /* Controllers */
  public static final CommandXboxController driver    = new CommandXboxController(0);
  public static final CommandXboxController copilot   = new CommandXboxController(1);
  public static final CommandXboxController buttonBox = new CommandXboxController(2);
  public static final CommandXboxController testing   = new CommandXboxController(3);
  public static final CommandXboxController sysID     = new CommandXboxController(4);

  /* Subsystems */
  public static final CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
  public static final Limelight s_LimelightPort = new Limelight(IDConstants.llPortName);
  public static final Limelight s_LimelightStbd = new Limelight(IDConstants.llStbdName);
  
  public static final Diffector s_Diffector = new Diffector();
  public static final Climber s_Climber = new Climber();
  public static final CoralManipulator s_Coral = new CoralManipulator();
  public static final AlgaeManipulator s_Algae = new AlgaeManipulator();
  public static final CANifierAccess s_Canifier = new CANifierAccess();
  public static final Rumbler s_Rumbler = new Rumbler(driver, copilot);

  /* Driver Control Axis */
  public static final int translationAxis = XboxController.Axis.kLeftY.value;
  public static final int strafeAxis      = XboxController.Axis.kLeftX.value;
  public static final int rotationAxis    = XboxController.Axis.kRightX.value;
  public static final int brakeAxis       = XboxController.Axis.kRightTrigger.value;

  /* Codriver Control Axis */
  public static final int manualClimberAxis            = XboxController.Axis.kLeftY.value;
  public static final int manualDiffectorElevationAxis = XboxController.Axis.kRightY.value;
  public static final int manualDiffectorRotationAxis  = XboxController.Axis.kRightX.value;

  /* Triggers */
  private static final Trigger unlockHeadingTrigger   = new Trigger(() -> Math.abs(driver.getRawAxis(rotationAxis)) > Constants.Control.stickDeadband);
  private static final Trigger cageDriveTrigger       = new Trigger(() -> headingState == HeadingStates.CAGE_LOCK);
  private static final Trigger scoreDriveTrigger      = new Trigger(() -> headingState == HeadingStates.REEF_LOCK);
  private static final Trigger stationDriveTrigger    = new Trigger(() -> headingState == HeadingStates.STATION_LOCK);
  private static final Trigger processorDriveTrigger  = new Trigger(() -> headingState == HeadingStates.PROCESSOR_LOCK);
  private static final Trigger autoScoreCancelTrigger = new Trigger
  (
    unlockHeadingTrigger.or
    (() -> 
      driver.getRawAxis(translationAxis) > Constants.Control.stickDeadband ||
      copilot.getRawAxis(manualDiffectorElevationAxis) > Constants.Control.manualDiffectorDeadband ||
      copilot.getRawAxis(manualDiffectorRotationAxis) > Constants.Control.manualDiffectorDeadband
    )
  );
//  private final Trigger driverLeftRumbleTrigger    = new Trigger(() -> s_Intake.getAlgaeState());
//  private final Trigger copilotLeftRumbleTrigger   = new Trigger(
//              () -> s_Intake.getAlgaeState() && (s_Diffector.getRelativeRotation() > 45 && s_Diffector.getRelativeRotation() < 315) ||
//              s_Intake.getAlgaeState() && (s_Diffector.getRelativeRotation() > 135 && s_Diffector.getRelativeRotation() < 225));
  //private final Trigger driverRightRumblTrigger = new Trigger(() -> );
  // TODO: Ready to score rumble
  //private final Trigger copliotRightRumbleTrigger = new Trigger(() -> s_Intake.climbReady() && s_Climber.climbReady() && s_Diffector.climbReady() );

  /* Control Modifiers */
  private static final BooleanSupplier algaeModifier = copilot.rightTrigger();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    swerveState = s_Swerve.getState();

    SmartDashboard.putBoolean("IgnoreFence", false);
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
        () -> true
      )
    );

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

          builder.addDoubleProperty("Robot Angle", () -> swerveState.Pose.getRotation().getRadians(), null);
        }
      }
    );

    // Configure button bindings
    configureDriverBindings();
    configureAutoDriveBindings();
    configureCopilotBindings();
    //configureTestBindings();
    configureRumbleBindings();
    configureManualBindings();

    s_Swerve.registerTelemetry(logger::telemeterize);
  }

  private void configureDriverBindings()
  {
    // Heading reset
    driver.start()
      .onTrue
      (
        Commands.runOnce
        (
          () -> 
          {
            Pigeon2 pigeon = s_Swerve.getPigeon2();

            pigeon.setYaw(FieldUtils.isRedAlliance() ? 0 : 180);
            s_Swerve.resetPose(new Pose2d(swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(pigeon.getYaw().getValueAsDouble()))));
          }
        )
      );
      
    /* Intake controls */
    driver.leftTrigger()
      .onTrue(s_Coral.setStatusCommand(CoralManipulatorStatus.DELIVERY_SMART)).onFalse(s_Coral.setStatusCommand(CoralManipulatorStatus.DEFAULT));
    driver.leftBumper()
      .onTrue(s_Algae.setStatusCommand(AlgaeManipulatorStatus.EJECT)).onFalse(s_Algae.setStatusCommand(AlgaeManipulatorStatus.EMPTY));

    /* Smart Intake and Auto Score controls */
    driver.rightBumper() // TODO: Intake is now part of Diffector system
      .whileTrue
      (
        new FunctionalCommand
        (
          () -> s_Diffector.setTargetPosition(Constants.DiffectorConstants.algaeIntakePosition), 
          () -> {if (s_Diffector.atPosition()) s_Algae.setAlgaeManipulatorStatus(AlgaeManipulatorStatus.INTAKE);}, 
          interrupted -> 
          {    
            s_Algae.setAlgaeManipulatorStatus(AlgaeManipulatorStatus.HOLDING);
            if (RobotContainer.algae)
              {s_Diffector.setTargetPosition(Constants.DiffectorConstants.algaeStowPosition);}
          }, 
          () -> false, 
          s_Diffector, s_Algae
        )
      );

    driver.back()
      .onTrue
      (
        AutoUtils.autoScoreSequenceCommand
        (
          s_Diffector, 
          s_Algae, 
          s_Coral, 
          () -> 
          {
            return 
            copilot.y().getAsBoolean() 
            ? 
            4 
            : 
            copilot.x().getAsBoolean() 
            ? 
            3 
            : 
            copilot.b().getAsBoolean() 
            ? 
            2 
            : 
            copilot.a().getAsBoolean() 
            ? 
            1 
            : 
            0;
          }, 
          driver.rightTrigger(), 
          () -> driver.getHID().getPOV(), 
          autoScoreCancelTrigger
        )
      );
  }

  private void configureAutoDriveBindings()
  {
    /* Heading lock state management */
    unlockHeadingTrigger.onTrue(Commands.runOnce(() -> headingState = HeadingStates.UNLOCKED));
    driver.y().onTrue(Commands.runOnce(() -> headingState = HeadingStates.CAGE_LOCK));
    driver.x().onTrue(Commands.runOnce(() -> headingState = HeadingStates.REEF_LOCK));
    driver.b().onTrue(Commands.runOnce(() -> headingState = HeadingStates.PROCESSOR_LOCK));
    driver.a().onTrue(Commands.runOnce(() -> headingState = HeadingStates.STATION_LOCK));

    /* 
      * Cage pathfinding controls 
      * Drives to the nearest reef face when the cage heading lock is active and a corresponding dpad direction is pressed 
      */ 
    cageDriveTrigger.and(driver.povUp())   .onTrue(AutoUtils.pathfindAndFollowCommand("cage2", driver.rightTrigger()));
    cageDriveTrigger.and(driver.povLeft()) .onTrue(AutoUtils.pathfindAndFollowCommand("cage3", driver.rightTrigger()));
    cageDriveTrigger.and(driver.povRight()).onTrue(AutoUtils.pathfindAndFollowCommand("cage1", driver.rightTrigger()));

    /* 
      * Station pathfinding controls 
      * Drives to the nearest coral station when the station heading lock is active and a corresponding dpad direction is pressed 
      */ 
    stationDriveTrigger.and(driver.povUp())   .onTrue(AutoUtils.pathfindToStationCommand(2, driver.rightTrigger()));
    stationDriveTrigger.and(driver.povLeft()) .onTrue(AutoUtils.pathfindToStationCommand(1, driver.rightTrigger()));
    stationDriveTrigger.and(driver.povRight()).onTrue(AutoUtils.pathfindToStationCommand(3, driver.rightTrigger()));

    /* 
      * Processor pathfinding control 
      * Runs when the processor heading lock is active and right is pressed on the dpad 
      */ 
    processorDriveTrigger.and(driver.povRight()).onTrue(AutoUtils.pathfindAndFollowCommand("p", driver.rightTrigger()));
    processorDriveTrigger.and(driver.povLeft()).onTrue(AutoUtils.pathfindAndFollowCommand("pOpp", driver.rightTrigger()));

    /* 
      * Reef and Net pathfinding controls 
      * Drives to the nearest reef face when the reef heading lock is active and a corresponding dpad direction is pressed 
      * Drives to the nearest net position when the scoring heading lock is active and down is pressed on the dpad
      */ 
    scoreDriveTrigger.and(driver.povUp())   .onTrue(AutoUtils.pathfindToReefCommand(DpadOptions.CENTRE, driver.rightTrigger()));
    scoreDriveTrigger.and(driver.povLeft()) .onTrue(AutoUtils.pathfindToReefCommand(DpadOptions.LEFT, driver.rightTrigger()));
    scoreDriveTrigger.and(driver.povRight()).onTrue(AutoUtils.pathfindToReefCommand(DpadOptions.RIGHT, driver.rightTrigger()));
    scoreDriveTrigger.and(driver.povDown()) .onTrue(AutoUtils.pathfindToBargeCommand(driver.rightTrigger()));

    /* 
      * Binds heading targetting commands to run while the appropriate trigger is active and the dpad isn't pressed
      * Does not need to check the rotation stick, as soon at the rotation stick is moved all drive triggers become false
      * Bind heading targeting commands to run while the appropriate head lock trigger is active and the dpad isn't pressed
      * Does not need to check the rotation stick, as soon as the rotation stick is moved all heading lock triggers become false 
      * (see start of this function)
      */
    cageDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        new TargetHeading
        (
          s_Swerve,
          Rotation2d.kZero, 
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
          Rotation2d.kZero,
          () -> swerveState.Pose.getY(),
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> driver.getRawAxis(brakeAxis),
          () -> !driver.leftStick().getAsBoolean()
        )
      );
  
    processorDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        new TargetHeadingProcessor
        (
          s_Swerve,
          Rotation2d.kCW_90deg, 
          () -> swerveState.Pose.getX(),
          Rotation2d.kCW_90deg,
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> driver.getRawAxis(brakeAxis),
          () -> !driver.leftStick().getAsBoolean()
        )
      );

    processorDriveTrigger
      .whileTrue
      (
        Commands.startEnd
        (
          () -> 
          {
            ArrayList<Pair<Translation2d, Translation2d>> bargeObstacle = new ArrayList<Pair<Translation2d, Translation2d>>()
            {
              {
                add(FieldUtils.isRedAlliance() ? FieldUtils.GeoFencing.redAllianceBargeDynamic : FieldUtils.GeoFencing.blueAllianceBargeDynamic);
              }
            };

            Pathfinding.setDynamicObstacles(bargeObstacle, swerveState.Pose.getTranslation());
          }, 
          () -> Pathfinding.setDynamicObstacles(new ArrayList<Pair<Translation2d, Translation2d>>(), swerveState.Pose.getTranslation())
        )
      );

    scoreDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        new TargetHeadingScore
        (
          s_Swerve, 
          90,
          () -> swerveState.Pose.getTranslation(),
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> driver.getRawAxis(brakeAxis),
          () -> !driver.leftStick().getAsBoolean()
        )
      );
  }

  private void configureCopilotBindings()
  {
    /* Climb controls */
    copilot.start()
      .onTrue(s_Climber.setStatusCommand(ClimberStatus.CLIMB));
    copilot.back()
      .onTrue
      (
        Commands.sequence
        (
          s_Climber.setStatusCommand(ClimberStatus.ACTIVE),
          s_Diffector.moveToCommand(Constants.DiffectorConstants.climbPosition)
        )
      ); //Deploys the climber        

    /* Game piece scoring and intake positions */
    copilot.y()
      .onTrue
      (
        Commands.either
        (
          s_Diffector.moveToCommand(Constants.DiffectorConstants.netPosition), 
          s_Diffector.coralScorePosCommand(4), 
          algaeModifier
        )
      );

    copilot.x()
      .onTrue
      (
        Commands.either
        (
          s_Diffector.algaeIntakePosCommand(false), 
          s_Diffector.coralScorePosCommand(3), 
          algaeModifier
        )
      );

    copilot.b()
      .onTrue
      (
        Commands.either
        (
          s_Diffector.algaeIntakePosCommand(true),
          s_Diffector.coralScorePosCommand(3), 
          algaeModifier
        )
      );

    copilot.a()
      .onTrue
      (
        Commands.either
        (
          s_Diffector.moveToCommand(Constants.DiffectorConstants.processorPosition), 
          s_Diffector.coralScorePosCommand(1), 
          algaeModifier
        )
      );

    /* Stow pos*/
    copilot.povUp()
      .onTrue
      (
        Commands.either
        (
          s_Diffector.moveToCommand(Constants.DiffectorConstants.algaeStowPosition), // Algae stow pos
          s_Diffector.moveToCommand(Constants.DiffectorConstants.coralStowPosition), // Coral stow pos
          algaeModifier
        )
      );

    /* Transfer pos */
    copilot.povDown()
      .onTrue
      (
        Commands.either
        (
          s_Diffector.moveToCommand(Constants.DiffectorConstants.algaeTransferPosition), // Algae transfer pos
          s_Diffector.moveToCommand(Constants.DiffectorConstants.coralTransferPosition), // Coral transfer pos
          algaeModifier
        )
      );

    /* Game piece intake position controls */
    copilot.rightBumper()
      .onTrue
      (
        Commands.either
        (
          s_Diffector.moveToCommand(Constants.DiffectorConstants.algaeIntakePosition), // Algae intake pos (ground)
          s_Diffector.moveToCommand(Constants.DiffectorConstants.coralIntakePosition), // Coral intake pos (clearance for station)
          algaeModifier
        )
      );
  }

  private void configureManualBindings()
  {
    /* Manual climber controls */
    copilot.axisMagnitudeGreaterThan(manualClimberAxis, Constants.Control.stickDeadband)
      .whileTrue(Commands.run(() -> s_Climber.manualOveride(copilot.getRawAxis(manualClimberAxis))))
      .onFalse(Commands.runOnce(() -> s_Climber.manualOveride(0)));

    /* Manual arm controls */
    copilot.axisMagnitudeGreaterThan(manualDiffectorElevationAxis, Constants.Control.manualDiffectorDeadband)
    .or(copilot.axisMagnitudeGreaterThan(manualDiffectorRotationAxis, Constants.Control.manualDiffectorDeadband))
      .whileTrue
      (
        s_Diffector.runEnd
        (
          () ->
          {
            double elevationBase = -copilot.getRawAxis(manualDiffectorElevationAxis);
            double rotationBase = -copilot.getRawAxis(manualDiffectorRotationAxis);

            double elevation = Math.abs(elevationBase) < 2 * Math.abs(rotationBase) ? 0 : elevationBase;
            double rotation = Math.abs(rotationBase) < 2 * Math.abs(elevationBase) ? 0 : rotationBase;
    
            s_Diffector.setManualDiffectorValues(elevation, rotation);
          }, 
          () -> s_Diffector.setManualDiffectorValues(0, 0)
        )
      );
    copilot.rightStick().whileTrue(Commands.run(() -> s_Diffector.unwind(), s_Diffector));

    /* Coral outtake controls */
    copilot.povLeft()
      .onTrue(s_Coral.setStatusCommand(CoralManipulatorStatus.DELIVERY_LEFT))
      .onFalse(s_Coral.setStatusCommand(CoralManipulatorStatus.DEFAULT));
    copilot.povRight()
      .onTrue(s_Coral.setStatusCommand(CoralManipulatorStatus.DELIVERY_RIGHT))
      .onFalse(s_Coral.setStatusCommand(CoralManipulatorStatus.DEFAULT));

    /* Algae intake/outtake controls */
    copilot.leftTrigger()
      .onTrue(s_Algae.setStatusCommand(AlgaeManipulatorStatus.INTAKE))
      .onFalse(s_Algae.setStatusCommand(AlgaeManipulatorStatus.HOLDING)); //Intake algae through manipulator
     copilot.leftBumper()
      .onTrue(s_Algae.setStatusCommand(AlgaeManipulatorStatus.EJECT))
      .onFalse(s_Algae.setStatusCommand(AlgaeManipulatorStatus.EMPTY)); //Ejects algae from manipulator
  }

  private void configureRumbleBindings()
  {
    /* Driver rumble bindings */ // TODO when reimplemented, replace command subclasses with Commands.runOnce()
    //driverLeftRumbleTrigger.onTrue(new SetRumble(s_Rumbler, Sides.DRIVER_RIGHT, "Intake Full"));
    // TODO: Driver Rightside Rumble: Ready To Score
    
    /* Copilot rumble bindings */
    //copilotLeftRumbleTrigger.onTrue(new SetRumble(s_Rumbler, Sides.COPILOT_LEFT, "Transfer Ready"));
    // TODO: copliotRightRumbleTrigger.onTrue(new SetRumble(s_Rumbler, Sides.COPILOT_RIGHT, "Climb Ready"));
  }
  
  @SuppressWarnings("unused")
  private void configureTestBindings()
  {}

  @SuppressWarnings("unused")
  private void configureButtonBoxBindings()
  {}

  public CommandSwerveDrivetrain getSwerve()
    {return s_Swerve;}

  public Limelight getLimelightPort()
    {return s_LimelightPort;}

  public Limelight getLimelightStbd()
    {return s_LimelightStbd;}

  public Command getAutoCommand()
  {
    // Gets the input string of command phrases, processes into a list of commands, and puts them into a sequential command group
    return AutoUtils.getCommandList(SmartDashboard.getString("Auto Input", Constants.Auto.defaultAuto), s_Diffector, s_Coral, s_Algae);
  }
}
