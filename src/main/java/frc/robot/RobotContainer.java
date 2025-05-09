package frc.robot;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.*;
import frc.robot.constants.*;
import frc.robot.constants.Constants.DiffectorConstants;
import frc.robot.constants.Constants.LEDStrip;
import frc.robot.constants.Constants.DiffectorConstants.Presets;
import frc.robot.subsystems.*;
import frc.robot.subsystems.AlgaeManipulator.Status;
import frc.robot.util.ArmPos;
import frc.robot.util.AutoUtils;
import frc.robot.util.Conversions;
import frc.robot.util.FieldUtils;
import frc.robot.util.SD;
import frc.robot.util.Triggers;
import frc.robot.util.leds.LightLayer;
import frc.robot.util.libraries.Telemetry;
import frc.robot.util.leds.LightLayer.Mode;
import frc.robot.util.leds.LightLayer.LayerType;

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
  public static final CommandGenericHID     buttonBox = new CommandGenericHID    (2);
  public static final CommandXboxController testing   = new CommandXboxController(3);
  public static final CommandXboxController sysID     = new CommandXboxController(4);

  /* Subsystems */
  public static final CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
  public static final Diffector s_Diffector            = new Diffector();
  public static final Climber s_Climber                = new Climber();
  public static final CoralManipulator s_Coral         = new CoralManipulator();
  public static final AlgaeManipulator s_Algae         = new AlgaeManipulator();
  public static final Limelight io_LimelightPort       = new Limelight(IDConstants.llPortName);
  public static final Limelight io_LimelightStbd       = new Limelight(IDConstants.llStbdName);
  public static final CANifierAccess io_Canifier       = new CANifierAccess();
  public static final RumbleRequester io_driverRight   = new RumbleRequester(driver, RumbleType.kRightRumble, SD.RUMBLE_D_R::put, SD.IO_RUMBLE_D::get);
  public static final RumbleRequester io_driverLeft    = new RumbleRequester(driver, RumbleType.kLeftRumble, SD.RUMBLE_D_L::put, SD.IO_RUMBLE_D::get);
  public static final RumbleRequester io_copilotRight  = new RumbleRequester(copilot, RumbleType.kRightRumble, SD.RUMBLE_C_R::put, SD.IO_RUMBLE_C::get);
  public static final RumbleRequester io_copilotLeft   = new RumbleRequester(copilot, RumbleType.kLeftRumble, SD.RUMBLE_C_L::put, SD.IO_RUMBLE_C::get);
  private final LEDRenderer io_Lights                  = new LEDRenderer();
  private LightLayer portStatusLayer                   = new LightLayer(s_Swerve, "PortStatus");
  private LightLayer stbdStatusLayer                   = new LightLayer(s_Swerve, "StbdStatus");
  private LightLayer haloPortLayer                     = new LightLayer(s_Swerve, "HaloPort");
  private LightLayer haloStbdLayer                     = new LightLayer(s_Swerve, "HaloStbd");
  private LightLayer allLEDsLayer                      = new LightLayer(s_Swerve, "AllLEDs");

  /* Driver Control Axis */
  public static final int translationAxis = Axis.kLeftY.value;
  public static final int strafeAxis      = Axis.kLeftX.value;
  public static final int rotationAxis    = Axis.kRightX.value;
  public static final int brakeAxis       = Axis.kRightTrigger.value;

  /* Codriver Control Axis */
  public static final int manualClimberAxis            = Axis.kLeftY.value;
  public static final int manualDiffectorElevationAxis = Axis.kRightY.value;
  public static final int manualDiffectorRotationAxis  = Axis.kRightX.value;

  /* Control Modifiers */
  private static final Trigger algaeModifier = copilot.rightTrigger();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    swerveState = s_Swerve.getState();

    s_Swerve.setDefaultCommand
    (
      new ManualDrive
      (
        s_Swerve, 
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis), 
        () -> -driver.getRawAxis(rotationAxis), 
        () -> driver.getRawAxis(brakeAxis),
        () -> true
      )
    );

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    // Configure button bindings
    configureDriverBindings();
    configureAutoDriveBindings();
    configureCopilotBindings();
    //configureRumbleBindings();
    configureManualBindings();
    //configureTestBindings();
    configureSDButtonBindings();
    configureMiscBindings();
    configureFenceBindings();

    s_Swerve.registerTelemetry(logger::telemeterize);
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
            Pigeon2 pigeon = s_Swerve.getPigeon2();

            pigeon.setYaw(FieldUtils.isRedAlliance() ? 0 : 180);
            s_Swerve.resetPose(new Pose2d(swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(pigeon.getYaw().getValueAsDouble()))));
            SD.ROTATION_KNOWN.put(false);
          }
        )
        .ignoringDisable(true)
        .withName("HeadingReset")
      );
      
    /* Outtake controls */
    driver.leftTrigger()
      .whileTrue
      (
        Commands.either
        (
          s_Algae.startEnd(() -> s_Algae.setStatus(AlgaeManipulator.Status.EJECT), () -> s_Algae.setStatus(AlgaeManipulator.Status.EMPTY)), 
          s_Coral.startEnd(() -> s_Coral.setStatus(CoralManipulator.Status.DELIVERY_SMART), () -> s_Coral.setStatus(CoralManipulator.Status.DEFAULT)), 
          () -> 
          {
            ArmPos target = s_Diffector.getRelativeTarget();
            return target.relativeEquals(Constants.DiffectorConstants.Presets.coral1Position);
          }
        )
        .withName("EjectCoral")
      );
    driver.leftTrigger()
      .and
      (
        () ->
        {
          var highCoralPositions = 
          List.of
          (
            Presets.coral4Position, 
            Presets.coral3Position
          ).stream();
          return
          (highCoralPositions.anyMatch(s_Diffector.getRelativeTarget()::relativeEquals));
        }
      )
      .onTrue
      (
        Commands.sequence
        (
          Commands.waitUntil(() -> !coral).withTimeout(0.05),
          s_Diffector.runOnce(() -> s_Diffector.goToAngle(0))
        )
      );
    driver.leftBumper()
      .onTrue(s_Algae.setStatusCommand(AlgaeManipulator.Status.EJECT)).onFalse(s_Algae.setStatusCommand(AlgaeManipulator.Status.EMPTY));

    /* Smart Intake and Auto Score controls */
    driver.rightBumper()
      .onTrue
      (
        Commands.either // Algae intake pos
        (
          s_Diffector.moveToCommand(DiffectorConstants.Presets.algaeIntakePosition.port()), 
          s_Diffector.moveToCommand(DiffectorConstants.Presets.algaeIntakePosition.stbd()), 
          () ->
          {
            double robotRotation = Conversions.mod(RobotContainer.swerveState.Pose.getRotation().getDegrees(), 360);
            return robotRotation < 180 ^ FieldUtils.isRedAlliance();
          }
        )
        .withName("SmartIntake")
      )
      .whileTrue
      (
        s_Algae.run(() -> {if (s_Diffector.atPosition()) s_Algae.setStatus(AlgaeManipulator.Status.INTAKE);})
        .withName("SmartIntake")
      )
      .onFalse
      (
        Commands.runOnce
        (() -> 
        {    
          s_Algae.setStatus(AlgaeManipulator.Status.HOLDING);
          if (RobotContainer.algae)
            {s_Diffector.setTargetPosition(DiffectorConstants.Presets.algaeStowPosition);}
        })
        .withName("SmartIntake")
      );

    /* driver.back()
      .onTrue
      (
        Commands.defer
        (
          () ->
          AutoUtils.autoScoreSequenceCommand
          (
            s_Diffector, 
            s_Algae, 
            s_Coral, 
            () -> 
            {
              return 
              copilot.y().getAsBoolean() ? 4 :
              copilot.x().getAsBoolean() ? 3 :
              copilot.b().getAsBoolean() ? 2 :
              copilot.a().getAsBoolean() ? 1 :
              0;
            }, 
            driver.rightTrigger(), 
            () -> driver.getHID().getPOV(), 
            Triggers.autoScoreCancelTrigger
          ),
          Set.of(s_Diffector, s_Algae, s_Coral)
        )
        .withName("AutoScore")
      ); */
  }

  private void configureAutoDriveBindings()
  {
    /* Heading lock state management */
    Triggers.unlockHeadingTrigger.or(driver.start()).onTrue(Commands.runOnce(() -> headingState = HeadingStates.UNLOCKED));
    driver.y().onTrue(Commands.runOnce(() -> headingState = HeadingStates.CAGE_LOCK));
    driver.x().onTrue(Commands.runOnce(() -> headingState = HeadingStates.REEF_LOCK));
    driver.b().onTrue(Commands.runOnce(() -> headingState = HeadingStates.PROCESSOR_LOCK));
    driver.a().onTrue(Commands.runOnce(() -> headingState = HeadingStates.STATION_LOCK));

    // ### Disabled most autodriving for display ###

    /* 
      * Cage pathfinding controls 
      * Drives to the nearest reef face when the cage heading lock is active and a corresponding dpad direction is pressed 
      */ 
    //Triggers.cageDriveTrigger.and(driver.povUp())   .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "cage2", driver.rightTrigger())));
    //Triggers.cageDriveTrigger.and(driver.povLeft()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "cage3", driver.rightTrigger())));
    //Triggers.cageDriveTrigger.and(driver.povRight()).and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "cage1", driver.rightTrigger())));
    //Triggers.cageDriveTrigger.and(driver.povDown()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getClimbPathName(), driver.rightTrigger())));

    /* 
      * Station pathfinding controls 
      * Drives to the nearest coral station when the station heading lock is active and a corresponding dpad direction is pressed 
      */ 
    //Triggers.stationDriveTrigger.and(driver.povUp())   .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getStationPathName(2), driver.rightTrigger())));
    //Triggers.stationDriveTrigger.and(driver.povLeft()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getStationPathName(1), driver.rightTrigger())));
    //Triggers.stationDriveTrigger.and(driver.povRight()).and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getStationPathName(3), driver.rightTrigger())));

    /* 
      * Processor pathfinding control 
      * Runs when the processor heading lock is active and right is pressed on the dpad 
      */ 
    //Triggers.processorDriveTrigger.and(driver.povRight()).and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "p", driver.rightTrigger())));
    //Triggers.processorDriveTrigger.and(driver.povLeft()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "pOpp", driver.rightTrigger())));

    /* 
      * Reef and Net pathfinding controls 
      * Drives to the nearest reef face when the reef heading lock is active and a corresponding dpad direction is pressed 
      * Drives to the nearest net position when the scoring heading lock is active and down is pressed on the dpad
      */ 
    Triggers.scoreDriveTrigger.and(driver.povUp())   .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getReefPathName(DpadOptions.CENTRE), driver.rightTrigger())));
    Triggers.scoreDriveTrigger.and(driver.povLeft()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getReefPathName(DpadOptions.LEFT), driver.rightTrigger())));
    Triggers.scoreDriveTrigger.and(driver.povRight()).and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getReefPathName(DpadOptions.RIGHT), driver.rightTrigger())));
    //Triggers.scoreDriveTrigger.and(driver.povDown()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.bargePathNameSup, driver.rightTrigger())));

    /* 
      * Binds heading targetting commands to run while the appropriate trigger is active and the dpad isn't pressed
      * Does not need to check the rotation stick, as soon at the rotation stick is moved all drive triggers become false
      * Bind heading targeting commands to run while the appropriate head lock trigger is active and the dpad isn't pressed
      * Does not need to check the rotation stick, as soon as the rotation stick is moved all heading lock triggers become false 
      * (see start of this function)
      */
    Triggers.cageDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        new TargetCageDrive
        (
          s_Swerve,
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          Rotation2d.kZero, 
          Rotation2d.kZero,
          () -> driver.getRawAxis(brakeAxis),
          () -> true
        )
        .withName("CageLock")
      );

    Triggers.stationDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        //new TargetStationDrive
        new HeadingLockedDrive
        (
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          Rotation2d.kZero,
          Rotation2d.k180deg,
          () -> driver.getRawAxis(brakeAxis),
          () -> true
        )
        .withName("StationLock")
      );
  
    Triggers.processorDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        new HeadingLockedDrive
        (
          s_Swerve,
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          Rotation2d.kZero,
          Rotation2d.kCCW_90deg,
          () -> driver.getRawAxis(brakeAxis),
          () -> true
        )
        .withName("ProcessorLock")
      );

    Triggers.scoreDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        //new TargetScoreDrive
        new HeadingLockedDrive
        (
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          Rotation2d.kZero,
          Rotation2d.kCW_90deg,
          () -> driver.getRawAxis(brakeAxis),
          () -> true
        )
        .withName("ScoreLock")
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
          s_Diffector.moveToCommand(Presets.climbPosition),
          s_Climber.setStatusCommand(Climber.Status.CLIMB),
          Commands.waitUntil(s_Climber::driverRumbleAngle),
          io_driverLeft.timedRequestCommand("Climb Drive", 0.25),
          Commands.waitUntil(s_Climber::offGround),
          Commands.runOnce(() -> headingState = HeadingStates.UNLOCKED)
        )
        .withName("Climb")
      );
      
    copilot.back()
      .onTrue
      (
        Commands.parallel
        (
          s_Diffector.moveAndWaitCommand(Presets.climbPosition),
          s_Climber.setStatusCommand(Climber.Status.ACTIVE).andThen(Commands.waitUntil(s_Climber::climbReady))
        )
        .andThen(io_copilotRight.timedRequestCommand("Climb Ready", 1))
        .withName("PrepareClimb")
      );

    copilot.back().or(copilot.start())
      .onTrue(Commands.runOnce(s_Climber::unlockClimb));

    /* Game piece scoring and intake positions */
    copilot.y()
      .onTrue
      (
        Commands.either
        (
          s_Diffector.moveToCommand(DiffectorConstants.Presets.netPosition), 
          s_Diffector.coralScorePosCommand(4), 
          algaeModifier
        )
        .withName("Level4")
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
        .withName("Level3")
      );

    copilot.b()
      .onTrue
      (
        Commands.either
        (
          s_Diffector.algaeIntakePosCommand(true),
          s_Diffector.coralScorePosCommand(2),
          algaeModifier
        )
        .withName("Level2")
      );

    copilot.a()
      .onTrue
      (
        Commands.either
        (
          Commands.either
          (
            s_Diffector.dualPosCommand(DiffectorConstants.Presets.processorPosition.stbd(), DiffectorConstants.Presets.processorPosition.port()), 
            s_Diffector.dualPosCommand(DiffectorConstants.Presets.processorPosition.port(), DiffectorConstants.Presets.processorPosition.stbd()), 
            () -> (swerveState.Pose.getX() >= FieldUtils.fieldLength/2 ^ FieldUtils.isRedAlliance())
          )
          .withName("Processor"),
          s_Diffector.coralScorePosCommand(1).withName("Coral1"), 
          algaeModifier
        )
      );

    copilot.povUp()
      .onTrue
      (
        Commands.either
        (
          s_Diffector.moveToCommand(DiffectorConstants.Presets.algaeStowPosition),
          s_Diffector.moveToCommand(DiffectorConstants.Presets.coralStowPosition), 
          algaeModifier
        )
        .withName("StowPos")
      );

    copilot.povDown().and(algaeModifier)
      .onTrue
      (
        Commands.either
        (
          s_Diffector.dualPosCommand(DiffectorConstants.Presets.algaeIntakePosition.port(), DiffectorConstants.Presets.algaeIntakePosition.stbd()), 
          s_Diffector.dualPosCommand(DiffectorConstants.Presets.algaeIntakePosition.stbd(), DiffectorConstants.Presets.algaeIntakePosition.port()), 
          () ->
          {
            double robotRotation = Conversions.mod(RobotContainer.swerveState.Pose.getRotation().getDegrees(), 360);
            return robotRotation < 180;
          }
        )
        .withName("AlgaeGroundIntake")
      );

    copilot.povDown().and(algaeModifier.negate())
      .onTrue(s_Coral.setStatusCommand(CoralManipulator.Status.WIGGLE));

    /* Game piece intake position controls */
    copilot.rightBumper()
      .onTrue
      (
        s_Diffector.defer
        (
          () -> s_Diffector.stationIntakePosCommand
          (
            () -> swerveState.Pose.getTranslation(), 
            algaeModifier
          )
        )
        .withName("CoralStation")
      );

    Triggers.atCoralStationTrigger.and(() -> !coral)
      .and
      (
        () ->
        {
          var coralIntakePositions = List.of(Presets.coralIntakePosition).stream();
          return 
          (coralIntakePositions.anyMatch(s_Diffector.getRelativeTarget()::relativeEquals));
        }
      )
      .onTrue(s_Coral.setStatusCommand(CoralManipulator.Status.INTAKE));

    Triggers.atCoralStationTrigger.and(() -> !algae)
      .and
      (
        () ->
        {
          var clawIntakePositions = List.of(Presets.coralClawPosition).stream();
          return 
          (clawIntakePositions.anyMatch(s_Diffector.getRelativeTarget()::relativeEquals));
        }
      )
      .onTrue(s_Algae.setStatusCommand(AlgaeManipulator.Status.INTAKE))
      .onFalse(s_Algae.setStatusCommand(AlgaeManipulator.Status.HOLDING).andThen(Commands.runOnce(() -> algae = true)));

    Triggers.algaeIntakePosTrigger
      .onTrue(s_Algae.setStatusCommand(Status.MANUAL_INTAKE))
      .onFalse(Commands.either(s_Algae.setStatusCommand(Status.HOLDING), s_Algae.setStatusCommand(Status.EMPTY), () -> algae));
  }

  private void configureManualBindings()
  {
    /* Manual climber controls */
    copilot.axisMagnitudeGreaterThan(manualClimberAxis, Constants.Control.stickDeadband)
      .whileTrue(s_Climber.run(() -> s_Climber.manualOveride(copilot.getRawAxis(manualClimberAxis))))
      .onFalse(s_Climber.runOnce(() -> s_Climber.manualOveride(0)));

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
    copilot.rightStick().whileTrue(s_Diffector.run(s_Diffector::unwind));

    /* Coral outtake controls */
    copilot.povLeft()
      .onTrue(s_Coral.setStatusCommand(CoralManipulator.Status.DELIVERY_LEFT))
      .onFalse(s_Coral.setStatusCommand(CoralManipulator.Status.DEFAULT));
    copilot.povRight()
      .onTrue(s_Coral.setStatusCommand(CoralManipulator.Status.DELIVERY_RIGHT))
      .onFalse(s_Coral.setStatusCommand(CoralManipulator.Status.DEFAULT));

    /* Algae intake/outtake controls */
    copilot.leftTrigger()
      .onTrue(s_Algae.setStatusCommand(AlgaeManipulator.Status.MANUAL_INTAKE))
      .onFalse(s_Algae.setStatusCommand(AlgaeManipulator.Status.HOLDING)); //Intake algae through manipulator
    copilot.leftBumper()
      .onTrue(s_Algae.setStatusCommand(AlgaeManipulator.Status.EJECT))
      .onFalse(s_Algae.setStatusCommand(AlgaeManipulator.Status.EMPTY)); //Ejects algae from manipulator
  }

  private void configureRumbleBindings()
  {
    /* Driver rumble bindings */
    io_driverLeft
      .addRumbleTrigger("Penalty Reef Zone", Triggers.opposingReefZoneTrigger.and(Triggers.usePenaltyRumbleTrigger))
      .addRumbleTrigger("Penalty Barge Zone", Triggers.opposingBargeZoneTrigger.and(Triggers.usePenaltyRumbleTrigger));
    io_driverRight.addRumbleTrigger( "Intaked Successfully", Triggers.driverRightRumbleTrigger);

    /* Copilot rumble bindings */
    io_copilotLeft.addRumbleTrigger("Intake Full", Triggers.copilotLeftRumbleTrigger);
    io_copilotRight.addRumbleTrigger("Diffector E-stopped", new Trigger(() -> SD.DIFF_ESTOP.get()));
    new Trigger(() -> Timer.getMatchTime() <= 6 && DriverStation.isTeleop())
      .onTrue
      (
        Commands.sequence
        (
          io_copilotRight.requestCommand(true, "Climb Alert"),
          Commands.waitUntil(copilot.start()),
          io_copilotRight.requestCommand(false, "Climb Alert")
        )
      );
  }

  private void configureTestBindings()
  {}

  private void configureSDButtonBindings()
  {
    /* ### Disable most uses of auto-driving for displays ###

    new Trigger(SD.IO_POSE_PATHFIND::button)
      .onTrue
      (
        s_Swerve.defer
        (
          () -> AutoBuilder.pathfindToPose
          (
            new Pose2d
            (
              SD.IO_POSE_X.get(), 
              SD.IO_POSE_Y.get(), 
              new Rotation2d(Units.degreesToRadians(SD.IO_POSE_R.get()))
            ), 
            Constants.Auto.defaultConstraints
          )
        )
      );
    
    */

    new Trigger(SD.IO_DIFF_GOTO::button)
      .onTrue
      (
        s_Diffector.defer
        (
          () -> s_Diffector.moveToCommand(new ArmPos(SD.IO_DIFF_ELEVATION.get(), SD.IO_DIFF_ANGLE.get()))
        )
      );
  }

  private void configureMiscBindings()
  {
    new Trigger(SD.DIFF_ESTOP::get)
      .and(() -> DriverStation.isAutonomous())
      .onTrue
      (
        Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> SD.DIFF_ESTOP.put(false)))
      );
  }

  private void initLED()
  { 
    portStatusLayer
      .setMode(Mode.STATICSEGMENT)
      .setStart(LEDStrip.portStatusStart)
      .setWidth(LEDStrip.portStatusWidth)
      .setType(LayerType.PROGRESS)
      .setPriority(5)
      .setColor(Color.kGreen, Color.kBlack)
      .setPeriod(0.2)
      .setBorder(false)
      .setReversed(true)
      .setProgressSupplier(() -> 
        MathUtil.interpolate(DiffectorGeometry.minZ, DiffectorGeometry.maxZ, s_Diffector.getElevation()));
    
    stbdStatusLayer
      .setMode(Mode.STATICSEGMENT)
      .setStart(LEDStrip.stbdStatusStart)
      .setWidth(LEDStrip.stbdStatusWidth)
      .setType(LayerType.PROGRESS)
      .setPriority(5)
      .setColor(Color.kGreen, Color.kBlack)
      .setPeriod(0.2)
      .setBorder(false)
      .setProgressSupplier(() -> 
        MathUtil.interpolate(DiffectorGeometry.minZ, DiffectorGeometry.maxZ, s_Diffector.getElevation()));
      

    haloPortLayer.setStart(LEDStrip.portHaloStart)
      .setWidth(LEDStrip.portHaloWidth)
      .setMode(Mode.STATICSEGMENT)
      .setType(LayerType.SOLID)
      .setPeriod(0.2)
      .setPriority(5)
      .setColor(Color.kYellow, Color.kBlack)
      .setSegments(10)
      .setBorder(false)
      .setReversed(true);

    haloStbdLayer.setStart(LEDStrip.stbdHaloStart)
      .setWidth(LEDStrip.stbdHaloWidth)
      .setMode(Mode.STATICSEGMENT)
      .setType(LayerType.SOLID)
      .setPeriod(0.2)
      .setPriority(5)
      .setColor(Color.kYellow, Color.kBlack)
      .setSegments(10)
      .setBorder(false);

    allLEDsLayer.setMode(Mode.WHOLESTRIP)
      .setStart(0)
      .setWidth(LEDStrip.lightsLen)
      .setType(LayerType.SOLID)
      .setPeriod(0.1)
      .setPriority(7)
      .setColor(Color.kBlack, Color.kRed)
      .setBorder(false);

    io_Lights
      .addLayer(portStatusLayer)
      .addLayer(stbdStatusLayer)
      .addLayer(haloPortLayer)
      .addLayer(haloStbdLayer)
      .addLayer(allLEDsLayer);

    Triggers.algaeLEDs.onTrue
    (
      Commands
      .runOnce
        (() -> {
          SD.STATE_LED_BAR.put("algae arm mode");
          portStatusLayer.setColor(Color.kDeepSkyBlue, Color.kBlack);
          stbdStatusLayer.setColor(Color.kDeepSkyBlue, Color.kBlack);
        })
    );

    Triggers.coralLEDs.onTrue
    (
      Commands
      .runOnce
        (() -> {
          SD.STATE_LED_BAR.put("coral arm mode");
          portStatusLayer.setColor(Color.kWhite, Color.kBlack);
          stbdStatusLayer.setColor(Color.kWhite, Color.kBlack);
        })
    );

    Triggers.manualControlLEDs
      .onTrue
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_BAR.put("manual arm mode");
          portStatusLayer.setColor(Color.kPurple, Color.kBlack);
          stbdStatusLayer.setColor(Color.kPurple, Color.kBlack);
        })
      );
      
      Triggers.eStopLEDs
      .onTrue
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_BAR.put("eStop arm mode");
          portStatusLayer.setType(LayerType.ALTERNATING);
          //portStatusLayer.setColor(Color.kBlack,Color.kRed);
          stbdStatusLayer.setType(LayerType.ALTERNATING);
          //stbdStatusLayer.setColor(Color.kBlack,Color.kRed);
        })
      )
      .onFalse
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_BAR.put("eStop cleared");
          portStatusLayer.setType(LayerType.PROGRESS);
          stbdStatusLayer.setType(LayerType.PROGRESS);
        })
      );

    Triggers.manualDriveLEDs
      .onTrue
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_HAL.put("manual");
          haloPortLayer
            .setType(LayerType.SOLID)
            .setColor(Color.kPurple,Color.kBlack);
          haloStbdLayer
            .setType(LayerType.SOLID)
            .setColor(Color.kPurple,Color.kBlack);
        })
      );

    Triggers.headingLockLEDs
      .onTrue
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_HAL.put("heading lock");
          haloPortLayer
            .setType(LayerType.SOLID)
            .setColor(Color.kOrange,Color.kBlack);
          haloStbdLayer
            .setType(LayerType.SOLID)
            .setColor(Color.kOrange,Color.kBlack);
        })
      );

    Triggers.pathFollowing.onTrue
    (
      Commands.runOnce
      (() -> {
        SD.STATE_LED_HAL.put("following path");
        haloPortLayer
          .setType(LayerType.ALTERNATING)
          .setColor(Color.kYellow,Color.kBlack);
        haloStbdLayer
          .setType(LayerType.ALTERNATING)
          .setColor(Color.kYellow,Color.kBlack);
      })
    );
    
    Triggers.pathTarget
    .and(Triggers.armAtTarget.negate())
    .onTrue
    (
      Commands.runOnce
      (() -> {
        SD.STATE_LED_HAL.put("at path target");
        haloPortLayer
          .setType(LayerType.SOLID)
          .setColor(Color.kYellow,Color.kBlack);
        haloStbdLayer
          .setType(LayerType.SOLID)
          .setColor(Color.kYellow,Color.kBlack);
      })
    );

    Triggers.pathTarget
    .and(Triggers.armAtTarget)
    .onTrue
    (
      Commands.runOnce
      (() -> {
        SD.STATE_LED_HAL.put("path and arm target");
        haloPortLayer
          .setType(LayerType.SOLID)
          .setColor(Color.kGreen,Color.kBlack);
        haloStbdLayer
          .setType(LayerType.SOLID)
          .setColor(Color.kGreen,Color.kBlack);
      })
    );

    Triggers.robotArmAndClimberAtTargetLEDs
      .onTrue
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_HAL.put("climb ready");
          haloPortLayer
            .setType(LayerType.SOLID)
            .setColor(Color.kGreen,Color.kBlack);
          haloStbdLayer
            .setType(LayerType.SOLID)
            .setColor(Color.kGreen,Color.kBlack);
        })
      );

    Triggers.intakeFullLEDs
      .onTrue
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_HAL.put("intake full");
          haloPortLayer
            .setType(LayerType.ALTERNATING)
            .setColor(Color.kGreen,Color.kBlack);
          haloStbdLayer
            .setType(LayerType.ALTERNATING)
            .setColor(Color.kGreen,Color.kBlack);
        })
      );

    Triggers.homeReefZoneTrigger
      .and(Triggers.pathTarget)
      .and(Triggers.armAtReefCoral)
      .and(() -> !coral)
      .onTrue
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_HAL.put("delivery success");
          haloPortLayer
            .setType(LayerType.ALTERNATING)
            .setColor(Color.kGreen,Color.kBlack);
          haloStbdLayer
            .setType(LayerType.ALTERNATING)
            .setColor(Color.kGreen,Color.kBlack);
        })
      );

    Triggers.homeReefZoneTrigger
      .and(Triggers.pathTarget)
      .and(Triggers.armAtReefAlgae)
      .and(() -> algae)
      .onTrue
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_HAL.put("algae intake full");
          haloPortLayer
            .setType(LayerType.ALTERNATING)
            .setColor(Color.kGreen,Color.kBlack);
          haloStbdLayer
            .setType(LayerType.ALTERNATING)
            .setColor(Color.kGreen,Color.kBlack);
        })
      );

    Triggers.timerClimbLEDs
      .onTrue
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_ALL.put("climb timer true");
          allLEDsLayer.setType(LayerType.ALTERNATING).setColor(Color.kRed,new Color(50, 0, 0));
        })
      )
      .onFalse
      (
        Commands.runOnce
        (() -> {
          SD.STATE_LED_ALL.put("climb timer false");
          allLEDsLayer.setType(LayerType.SOLID).setColor(Color.kBlack,Color.kRed);
        })
      );


//    allLEDsLayer.setPriority(-(allLEDsLayer.getPriority()));
  } 
  
  public Command getAutoCommand()
  {
    // Gets the input string of command phrases, processes into a list of commands, and puts them into a sequential command group
    return AutoUtils.getCommandList(SD.IO_AUTO.get(), s_Diffector, s_Coral, s_Algae);
  } 

  private void configureFenceBindings()
  {
    new Trigger(() -> SD.IO_FENCE_SET.button()).onTrue(Commands.runOnce(() -> 
    {
      FieldUtils.GeoFencing.field.updateBox
      (
        SD.IO_FENCE_XA.get() + 4.5,
        SD.IO_FENCE_YA.get() + 4,
        SD.IO_FENCE_XB.get() + 4.5,
        SD.IO_FENCE_YB.get() + 4
      );
      renderFieldWall();
    }));

    new Trigger(() -> SD.IO_FENCE_XAP.button())
        .onTrue(Commands.runOnce(() -> 
        {
          FieldUtils.GeoFencing.field.contractBox(0.2, 0);
          SD.IO_FENCE_XA.put(SD.IO_FENCE_XA.get() + 0.2);
          renderFieldWall();
        }));

    new Trigger(() -> SD.IO_FENCE_XAM.button())
        .onTrue(Commands.runOnce(() -> 
        {
          FieldUtils.GeoFencing.field.expandBox(-0.2, 0);
          SD.IO_FENCE_XA.put(SD.IO_FENCE_XA.get() - 0.2);
          renderFieldWall();
        }));

    new Trigger(() -> SD.IO_FENCE_YAP.button())
        .onTrue(Commands.runOnce(() -> 
        {
          FieldUtils.GeoFencing.field.contractBox(0, 0.2);
          SD.IO_FENCE_YA.put(SD.IO_FENCE_YA.get() + 0.2);
          renderFieldWall();
        }));

    new Trigger(() -> SD.IO_FENCE_YAM.button())
        .onTrue(Commands.runOnce(() -> 
        {
          FieldUtils.GeoFencing.field.expandBox(0, -0.2);
          SD.IO_FENCE_YA.put(SD.IO_FENCE_YA.get() - 0.2);
          renderFieldWall();
        }));

    new Trigger(() -> SD.IO_FENCE_XBP.button())
        .onTrue(Commands.runOnce(() -> 
        {
          FieldUtils.GeoFencing.field.expandBox(0.2, 0);
          SD.IO_FENCE_XB.put(SD.IO_FENCE_XB.get() + 0.2);
          renderFieldWall();
        }));

    new Trigger(() -> SD.IO_FENCE_XBM.button())
        .onTrue(Commands.runOnce(() -> 
        {
          FieldUtils.GeoFencing.field.contractBox(-0.2, 0);
          SD.IO_FENCE_XB.put(SD.IO_FENCE_XB.get() - 0.2);
          renderFieldWall();
        }));

    new Trigger(() -> SD.IO_FENCE_YBP.button())
        .onTrue(Commands.runOnce(() -> 
        {
          FieldUtils.GeoFencing.field.expandBox(0, 0.2);
          SD.IO_FENCE_YB.put(SD.IO_FENCE_YB.get() + 0.2);
          renderFieldWall();
        }));

    new Trigger(() -> SD.IO_FENCE_YBM.button())
        .onTrue(Commands.runOnce(() -> 
        {
          FieldUtils.GeoFencing.field.contractBox(0, -0.2);
          SD.IO_FENCE_YB.put(SD.IO_FENCE_YB.get() - 0.2);
          renderFieldWall();
        }));

  }

  private void renderFieldWall()
  {
    s_Swerve.field.getObject("Corner1").setPose(SD.IO_FENCE_XA.get() + 4.5, SD.IO_FENCE_YA.get() + 4, Rotation2d.kZero);
    s_Swerve.field.getObject("Corner2").setPose(SD.IO_FENCE_XA.get() + 4.5, SD.IO_FENCE_YB.get() + 4, Rotation2d.kZero);
    s_Swerve.field.getObject("Corner3").setPose(SD.IO_FENCE_XB.get() + 4.5, SD.IO_FENCE_YA.get() + 4, Rotation2d.kZero);
    s_Swerve.field.getObject("Corner4").setPose(SD.IO_FENCE_XB.get() + 4.5, SD.IO_FENCE_YB.get() + 4, Rotation2d.kZero);
  }
}
