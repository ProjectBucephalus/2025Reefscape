package frc.robot;

import java.util.ArrayList;
import java.util.Set;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.*;
import frc.robot.constants.*;
import frc.robot.constants.Constants.DiffectorConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Rumbler.Sides;
import frc.robot.util.*;
import frc.robot.util.leds.LightLayer;
import frc.robot.util.leds.LightLayer.*;
import frc.robot.util.libraries.Telemetry;

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
  public static final CommandSwerveDrivetrain swerve  = TunerConstants.createDrivetrain();
  public static final Limelight limelightPort         = new Limelight(IDConstants.llPortName);
  public static final Limelight limelightStbd         = new Limelight(IDConstants.llStbdName);
  public static final Diffector diffector             = new Diffector();
  public static final Climber climber                 = new Climber();
  public static final CoralManipulator coralManip     = new CoralManipulator();
  public static final AlgaeManipulator algaeManip     = new AlgaeManipulator();
  public static final CANifierAccess canifier         = new CANifierAccess();
  public static final Rumbler rumbler                 = new Rumbler(driver, copilot);
  private final LEDRenderer lights                    = new LEDRenderer();
  private LightLayer progressLayer                    = new LightLayer(swerve, "Progress");
  private LightLayer statusLayer                      = new LightLayer(swerve, "Status");
  private LightLayer reefPointerLayer                 = new LightLayer(swerve, "ReefPointer");
  private LightLayer processorPointerLayer            = new LightLayer(swerve, "ProcPointer");

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
  private final Trigger unlockHeadingTrigger   = new Trigger(() -> Math.abs(driver.getRawAxis(rotationAxis)) > Constants.Control.stickDeadband);
  private final Trigger cageDriveTrigger       = new Trigger(() -> headingState == HeadingStates.CAGE_LOCK);
  private final Trigger scoreDriveTrigger      = new Trigger(() -> headingState == HeadingStates.REEF_LOCK);
  private final Trigger stationDriveTrigger    = new Trigger(() -> headingState == HeadingStates.STATION_LOCK);
  private final Trigger processorDriveTrigger  = new Trigger(() -> headingState == HeadingStates.PROCESSOR_LOCK);
  private final Trigger autoScoreCancelTrigger = new Trigger
  (
    unlockHeadingTrigger.or
    (() -> 
      driver.getRawAxis(translationAxis) > Constants.Control.stickDeadband ||
      copilot.getRawAxis(manualDiffectorElevationAxis) > Constants.Control.manualDiffectorDeadband ||
      copilot.getRawAxis(manualDiffectorRotationAxis) > Constants.Control.manualDiffectorDeadband
    )
  );
  private final Trigger driverLeftRumbleTrigger = new Trigger(() -> 
  coralManip.getStatus() == CoralManipulator.Status.INTAKE && (diffector.getRelativeRotation() > 45 && diffector.getRelativeRotation() < 315) ||
  algaeManip.getStatus() == AlgaeManipulator.Status.HOLDING && (diffector.getRelativeRotation() > 135 && diffector.getRelativeRotation() < 225));
  //private final Trigger copilotLeftRumbleTrigger   = new Trigger(() -> funnel);
  private final Trigger driverRightRumbleTrigger = new Trigger(() -> algaeManip.getStatus() == AlgaeManipulator.Status.HOLDING);
  private final Trigger copliotRightRumbleTrigger = new Trigger(() -> climber.isUnlocked() && diffector.climbReady() );

  /* Control Modifiers */
  private static final BooleanSupplier algaeModifier = copilot.rightTrigger();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    swerveState = swerve.getState();

    SD.IO_GEOFENCE.init();
    swerve.setDefaultCommand
    (
      new ManualDrive
      (
        swerve, 
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis), 
        () -> -driver.getRawAxis(rotationAxis), 
        () -> driver.getRawAxis(brakeAxis),
        () -> true
      )
    );

    SD.IO_AUTO.init();
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    // Configure button bindings
    configureDriverBindings();
    configureAutoDriveBindings();
    configureCopilotBindings();
    //configureTestBindings();
    configureRumbleBindings();
    configureManualBindings();

    swerve.registerTelemetry(logger::telemeterize);
    initLED();
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
            Pigeon2 pigeon = swerve.getPigeon2();

            pigeon.setYaw(FieldUtils.isRedAlliance() ? 0 : 180);
            swerve.resetPose(new Pose2d(swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(pigeon.getYaw().getValueAsDouble()))));
          }
        )
      );
      
    /* Outtake controls */
    driver.leftTrigger()
      .onTrue
      (
        Commands.either
        (
          algaeManip.startEnd(() -> algaeManip.setStatus(AlgaeManipulator.Status.EJECT), () -> algaeManip.setStatus(AlgaeManipulator.Status.EMPTY)), 
          coralManip.startEnd(() -> coralManip.setStatus(CoralManipulator.Status.DELIVERY_SMART), () -> coralManip.setStatus(CoralManipulator.Status.DEFAULT)), 
          () -> 
          {
            Translation2d target = diffector.getRelativeTarget();
            return target.equals(Constants.DiffectorConstants.Presets.coral1PortPosition) || target.equals(Constants.DiffectorConstants.Presets.coral1StbdPosition);
          }
        )
      );
    driver.leftBumper()
      .onTrue(algaeManip.setStatusCommand(AlgaeManipulator.Status.EJECT)).onFalse(algaeManip.setStatusCommand(AlgaeManipulator.Status.EMPTY));

    /* Smart Intake and Auto Score controls */
    driver.rightBumper()
      .whileTrue
      (
        diffector.runOnce(() -> diffector.setTargetPosition(DiffectorConstants.Presets.algaeIntakePortPosition))
        .andThen(algaeManip.run(() -> {if (diffector.atPosition()) algaeManip.setStatus(AlgaeManipulator.Status.INTAKE);}))
        .finallyDo
        (
          () -> 
          {    
            algaeManip.setStatus(AlgaeManipulator.Status.HOLDING);
            if (RobotContainer.algae)
              {diffector.setTargetPosition(DiffectorConstants.Presets.algaeStowPosition);}
          }
        )
      );

    driver.back()
      .onTrue
      (
        Commands.defer
        (
          () ->
          AutoUtils.autoScoreSequenceCommand
          (
            diffector, 
            algaeManip, 
            coralManip, 
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
          ),
          Set.of(diffector, algaeManip, coralManip)
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
    cageDriveTrigger.and(driver.povUp())   .onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "cage2", driver.rightTrigger())));
    cageDriveTrigger.and(driver.povLeft()) .onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "cage3", driver.rightTrigger())));
    cageDriveTrigger.and(driver.povRight()).onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "cage1", driver.rightTrigger())));

    /* 
      * Station pathfinding controls 
      * Drives to the nearest coral station when the station heading lock is active and a corresponding dpad direction is pressed 
      */ 
    stationDriveTrigger.and(driver.povUp())   .onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getStationPathName(2), driver.rightTrigger())));
    stationDriveTrigger.and(driver.povLeft()) .onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getStationPathName(1), driver.rightTrigger())));
    stationDriveTrigger.and(driver.povRight()).onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getStationPathName(3), driver.rightTrigger())));

    /* 
      * Processor pathfinding control 
      * Runs when the processor heading lock is active and right is pressed on the dpad 
      */ 
    processorDriveTrigger.and(driver.povRight()).onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "p", driver.rightTrigger())));
    processorDriveTrigger.and(driver.povLeft()) .onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "pOpp", driver.rightTrigger())));

    /* 
      * Reef and Net pathfinding controls 
      * Drives to the nearest reef face when the reef heading lock is active and a corresponding dpad direction is pressed 
      * Drives to the nearest net position when the scoring heading lock is active and down is pressed on the dpad
      */ 
    scoreDriveTrigger.and(driver.povUp())   .onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getReefPathName(DpadOptions.CENTRE), driver.rightTrigger())));
    scoreDriveTrigger.and(driver.povLeft()) .onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getReefPathName(DpadOptions.LEFT), driver.rightTrigger())));
    scoreDriveTrigger.and(driver.povRight()).onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getReefPathName(DpadOptions.RIGHT), driver.rightTrigger())));
    scoreDriveTrigger.and(driver.povDown()) .onTrue(swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getBargePathName(), driver.rightTrigger())));

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
        new HeadingLockedDrive
        (
          swerve,
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          Rotation2d.kZero, 
          Rotation2d.kZero,
          () -> driver.getRawAxis(brakeAxis),
          () -> true
        )
      );

    stationDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        new TargetStationDrive
        (
          swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          Rotation2d.kCW_90deg, // TODO: Need to have bot facing drivers, and invert arm positions accordingly
          () -> driver.getRawAxis(brakeAxis),
          () -> true
        )
      );
  
    processorDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        new TargetProcessorDrive
        (
          swerve,
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          Rotation2d.kCW_90deg, 
          Rotation2d.kCW_90deg,
          () -> driver.getRawAxis(brakeAxis),
          () -> true
        )
      );

    processorDriveTrigger
      .whileTrue
      (
        Commands.startEnd
        (
          () -> 
          {
            ArrayList<Pair<Translation2d, Translation2d>> bargeObstacle = new ArrayList<Pair<Translation2d, Translation2d>>();
            bargeObstacle.add(FieldUtils.isRedAlliance() ? FieldUtils.GeoFencing.redAllianceBargeDynamic : FieldUtils.GeoFencing.blueAllianceBargeDynamic);

            Pathfinding.setDynamicObstacles(bargeObstacle, swerveState.Pose.getTranslation());
          }, 
          () -> Pathfinding.setDynamicObstacles(new ArrayList<Pair<Translation2d, Translation2d>>(), swerveState.Pose.getTranslation())
        )
      );

    scoreDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        new TargetScoreDrive
        (
          swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          Rotation2d.kCCW_90deg,
          () -> driver.getRawAxis(brakeAxis),
          () -> true
        )
      );
  }

  private void configureCopilotBindings()
  {
    /* Climb controls */
    copilot.start()
      .onTrue
      (
        Commands.sequence
        (
          diffector.moveAndWaitCommand(DiffectorConstants.Presets.climbPosition),
          climber.setStatusCommand(Climber.Status.CLIMB)
        )
      );  
    copilot.back()
      .onTrue
      (
        Commands.sequence
        (
          diffector.moveAndWaitCommand(DiffectorConstants.Presets.climbSafePosition),
          climber.setStatusCommand(Climber.Status.ACTIVE)
        )
      );  

    /* Game piece scoring and intake positions */
    copilot.y()
      .onTrue
      (
        Commands.either
        (
          diffector.moveToCommand(DiffectorConstants.Presets.netPosition), 
          diffector.coralScorePosCommand(4), 
          algaeModifier
        )
      );

    copilot.x()
      .onTrue
      (
        Commands.either
        (
          diffector.algaeIntakePosCommand(false), 
          diffector.coralScorePosCommand(3), 
          algaeModifier
        )
      );

    copilot.b()
      .onTrue
      (
        Commands.either
        (
          diffector.algaeIntakePosCommand(true),
          diffector.coralScorePosCommand(2),
          algaeModifier
        )
      );

    copilot.a()
      .onTrue
      (
        Commands.either
        (
          diffector.moveToCommand(DiffectorConstants.Presets.processorPosition), 
          diffector.coralScorePosCommand(1), 
          algaeModifier
        )
      );

    /* Stow pos*/
    copilot.povUp()
      .onTrue
      (
        Commands.either
        (
          diffector.moveToCommand(DiffectorConstants.Presets.algaeStowPosition), // Algae stow pos
          diffector.moveToCommand(DiffectorConstants.Presets.coralStowPosition), // Coral stow pos
          algaeModifier
        )
      );

    /* Transfer pos */
    copilot.povDown()
      .onTrue
      (
        Commands.either
        (
          diffector.moveToCommand(DiffectorConstants.Presets.algaeIntakePortPosition), // Algae intake pos
          diffector.coralScorePosCommand(0), // Coral score level 1 with coral manipulator
          algaeModifier
        )
      );

    /* Game piece intake position controls */
    copilot.rightBumper()
      .onTrue
      (
        Commands.either
        (
          diffector.moveToCommand(DiffectorConstants.Presets.coralClawPortPosition), // Algae intake pos (ground)
          diffector.moveToCommand(DiffectorConstants.Presets.coralIntakePortPosition), // Coral intake pos (clearance for station)
          algaeModifier
        )
      );
  }

  private void configureManualBindings()
  {
    /* Manual climber controls */
    copilot.axisMagnitudeGreaterThan(manualClimberAxis, Constants.Control.stickDeadband)
      .whileTrue(climber.run(() -> climber.manualOveride(copilot.getRawAxis(manualClimberAxis))))
      .onFalse(climber.runOnce(() -> climber.manualOveride(0)));

    /* Manual arm controls */
    copilot.axisMagnitudeGreaterThan(manualDiffectorElevationAxis, Constants.Control.manualDiffectorDeadband)
    .or(copilot.axisMagnitudeGreaterThan(manualDiffectorRotationAxis, Constants.Control.manualDiffectorDeadband))
      .whileTrue
      (
        diffector.runEnd
        (
          () ->
          {
            double elevationBase = -copilot.getRawAxis(manualDiffectorElevationAxis);
            double rotationBase = -copilot.getRawAxis(manualDiffectorRotationAxis);

            double elevation = Math.abs(elevationBase) < 2 * Math.abs(rotationBase) ? 0 : elevationBase;
            double rotation = Math.abs(rotationBase) < 2 * Math.abs(elevationBase) ? 0 : rotationBase;
    
            diffector.setManualDiffectorValues(elevation, rotation);
          }, 
          () -> diffector.setManualDiffectorValues(0, 0)
        )
      );
    copilot.rightStick().whileTrue(diffector.run(() -> diffector.unwind()));

    /* Coral outtake controls */
    copilot.povLeft()
      .onTrue(coralManip.setStatusCommand(CoralManipulator.Status.DELIVERY_LEFT))
      .onFalse(coralManip.setStatusCommand(CoralManipulator.Status.DEFAULT));
    copilot.povRight()
      .onTrue(coralManip.setStatusCommand(CoralManipulator.Status.DELIVERY_RIGHT))
      .onFalse(coralManip.setStatusCommand(CoralManipulator.Status.DEFAULT));

    /* Algae intake/outtake controls */
    copilot.leftTrigger()
      .onTrue(algaeManip.setStatusCommand(AlgaeManipulator.Status.MANUAL_INTAKE))
      .onFalse(algaeManip.setStatusCommand(AlgaeManipulator.Status.HOLDING)); //Intake algae through manipulator
    copilot.leftBumper()
      .onTrue(algaeManip.setStatusCommand(AlgaeManipulator.Status.EJECT))
      .onFalse(algaeManip.setStatusCommand(AlgaeManipulator.Status.EMPTY)); //Ejects algae from manipulator
  }

  private void configureRumbleBindings()
  {
    /* Driver rumble bindings */
    driverLeftRumbleTrigger.onTrue(rumbler.runOnce(() -> rumbler.addRequest(Sides.DRIVER_RIGHT, "Ready to Score")));
    driverRightRumbleTrigger.onTrue(rumbler.runOnce(() -> rumbler.addRequest(Sides.DRIVER_LEFT, "Pickup Waiting")));

    /* Copilot rumble bindings */
    //copilotLeftRumbleTrigger.onTrue(rumbler.runOnce(() -> rumbler.addRequest(Sides.COPILOT_LEFT, "Intake Full")));
    copliotRightRumbleTrigger.onTrue(rumbler.runOnce(() -> rumbler.addRequest(Sides.COPILOT_RIGHT, "Climb Ready")));
  }

  private void initLED()
  {
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
    statusLayer.setTarget(new Translation2d(1.0,FieldUtils.fieldWidth));

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
    processorPointerLayer.setTarget(FieldUtils.DriverFieldRefs.driverRed1);

    lights.addLayer(progressLayer);
    lights.addLayer(statusLayer);
    lights.addLayer(reefPointerLayer);
    lights.addLayer(processorPointerLayer);

  }

  public Command getAutoCommand()
  {
    // Gets the input string of command phrases, processes into a list of commands, and puts them into a sequential command group
    return AutoUtils.getCommandList(SD.IO_AUTO.get(), diffector, coralManip, algaeManip);
  }
}
