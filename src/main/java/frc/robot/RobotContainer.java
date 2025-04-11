package frc.robot;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
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
  public static final CommandXboxController buttonBox = new CommandXboxController(2);
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
    configureRumbleBindings();
    configureManualBindings();
    //configureTestBindings();

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
      .whileTrue
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
        .andThen(s_Algae.run(() -> {if (s_Diffector.atPosition()) s_Algae.setStatus(AlgaeManipulator.Status.INTAKE);}))
        .finallyDo
        (
          () -> 
          {    
            s_Algae.setStatus(AlgaeManipulator.Status.HOLDING);
            if (RobotContainer.algae)
              {s_Diffector.setTargetPosition(DiffectorConstants.Presets.algaeStowPosition);}
          }
        )
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
    Triggers.unlockHeadingTrigger.onTrue(Commands.runOnce(() -> headingState = HeadingStates.UNLOCKED));
    driver.y().onTrue(Commands.runOnce(() -> headingState = HeadingStates.CAGE_LOCK));
    driver.x().onTrue(Commands.runOnce(() -> headingState = HeadingStates.REEF_LOCK));
    driver.b().onTrue(Commands.runOnce(() -> headingState = HeadingStates.PROCESSOR_LOCK));
    driver.a().onTrue(Commands.runOnce(() -> headingState = HeadingStates.STATION_LOCK));

    /* 
      * Cage pathfinding controls 
      * Drives to the nearest reef face when the cage heading lock is active and a corresponding dpad direction is pressed 
      */ 
    Triggers.cageDriveTrigger.and(driver.povUp())   .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "cage2", driver.rightTrigger())));
    Triggers.cageDriveTrigger.and(driver.povLeft()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "cage3", driver.rightTrigger())));
    Triggers.cageDriveTrigger.and(driver.povRight()).and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "cage1", driver.rightTrigger())));
    Triggers.cageDriveTrigger.and(driver.povDown()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getClimbPathName(), driver.rightTrigger())));

    /* 
      * Station pathfinding controls 
      * Drives to the nearest coral station when the station heading lock is active and a corresponding dpad direction is pressed 
      */ 
    Triggers.stationDriveTrigger.and(driver.povUp())   .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getStationPathName(2), driver.rightTrigger())));
    Triggers.stationDriveTrigger.and(driver.povLeft()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getStationPathName(1), driver.rightTrigger())));
    Triggers.stationDriveTrigger.and(driver.povRight()).and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getStationPathName(3), driver.rightTrigger())));

    /* 
      * Processor pathfinding control 
      * Runs when the processor heading lock is active and right is pressed on the dpad 
      */ 
    Triggers.processorDriveTrigger.and(driver.povRight()).and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "p", driver.rightTrigger())));
    Triggers.processorDriveTrigger.and(driver.povLeft()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(() -> "pOpp", driver.rightTrigger())));

    /* 
      * Reef and Net pathfinding controls 
      * Drives to the nearest reef face when the reef heading lock is active and a corresponding dpad direction is pressed 
      * Drives to the nearest net position when the scoring heading lock is active and down is pressed on the dpad
      */ 
    Triggers.scoreDriveTrigger.and(driver.povUp())   .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getReefPathName(DpadOptions.CENTRE), driver.rightTrigger())));
    Triggers.scoreDriveTrigger.and(driver.povLeft()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getReefPathName(DpadOptions.LEFT), driver.rightTrigger())));
    Triggers.scoreDriveTrigger.and(driver.povRight()).and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getReefPathName(DpadOptions.RIGHT), driver.rightTrigger())));
    Triggers.scoreDriveTrigger.and(driver.povDown()) .and(Triggers.allowAutoDriveTrigger).onTrue(s_Swerve.defer(() -> AutoUtils.pathfindAndFollowCommand(AutoUtils.getBargePathName(), driver.rightTrigger())));

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
        new TargetStationDrive
        (
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
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
          Rotation2d.kCW_90deg, 
          Rotation2d.kCW_90deg,
          () -> driver.getRawAxis(brakeAxis),
          () -> true
        )
        .withName("ProcessorLock")
      );

    Triggers.scoreDriveTrigger.and(driver.povCenter())
      .whileTrue
      (
        new TargetScoreDrive
        (
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          Rotation2d.kCCW_90deg,
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
            s_Diffector.moveToCommand(DiffectorConstants.Presets.processorPosition.stbd()), 
            s_Diffector.moveToCommand(DiffectorConstants.Presets.processorPosition.port()), 
            () -> swerveState.Pose.getX() >= 8.774
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
          s_Diffector.moveToCommand(DiffectorConstants.Presets.algaeIntakePosition.port()), 
          s_Diffector.moveToCommand(DiffectorConstants.Presets.algaeIntakePosition.stbd()), 
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
  }

  private void configureTestBindings()
  {
    testing.y().onTrue(s_Diffector.moveToCommand(new ArmPos(1.5, 90)));
    testing.a().onTrue(s_Diffector.moveToCommand(new ArmPos(0.5, 90)));
    testing.povUp().onTrue(s_Diffector.moveToCommand(new ArmPos(1, 0)));
    testing.povRight().onTrue(s_Diffector.moveToCommand(new ArmPos(1, 90)));
    testing.povDown().onTrue(s_Diffector.moveToCommand(new ArmPos(1, 180)));
    testing.povLeft().onTrue(s_Diffector.moveToCommand(new ArmPos(1, 270)));
  }

  private void initLED()
  { 
    portStatusLayer.setSegments(6);
    portStatusLayer.setMode(Mode.STATICSEGMENT);
    portStatusLayer.setStart(LEDStrip.portStatusStart);
    portStatusLayer.setWidth(LEDStrip.portStatusWidth);
    portStatusLayer.setType(LayerType.STATUS);
    portStatusLayer.setPriority(1);
    portStatusLayer.setColor(Color.kBlack, Color.kTeal);
    portStatusLayer.setPeriod(0.2);
    portStatusLayer.setBorder(false);

    stbdStatusLayer.setSegments(6);
    stbdStatusLayer.setMode(Mode.STATICSEGMENT);
    stbdStatusLayer.setStart(LEDStrip.stbdStatusStart);
    stbdStatusLayer.setWidth(LEDStrip.stbdStatusWidth);
    stbdStatusLayer.setType(LayerType.STATUS);
    stbdStatusLayer.setPriority(1);
    stbdStatusLayer.setColor(Color.kBlack, Color.kTeal);
    stbdStatusLayer.setReversed(true);
    stbdStatusLayer.setPeriod(0.2);
    stbdStatusLayer.setBorder(false);

    haloPortLayer.setStart(LEDStrip.portHaloStart);
    haloPortLayer.setWidth(LEDStrip.portHaloWidth);
    haloPortLayer.setMode(Mode.STATICSEGMENT);
    haloPortLayer.setType(LayerType.SOLID);
    haloPortLayer.setPeriod(0.2);
    haloPortLayer.setPriority(1);
    haloPortLayer.setColor(Color.kYellow, Color.kBlack);
    haloPortLayer.setSegments(10);
    haloPortLayer.setBorder(false);

    haloStbdLayer.setStart(LEDStrip.stbdHaloStart);
    haloStbdLayer.setWidth(LEDStrip.portHaloWidth);
    haloStbdLayer.setMode(Mode.STATICSEGMENT);
    haloStbdLayer.setType(LayerType.SOLID);
    haloStbdLayer.setPeriod(0.2);
    haloStbdLayer.setPriority(1);
    haloStbdLayer.setColor(Color.kYellow, Color.kBlack);
    haloStbdLayer.setSegments(10);
    haloStbdLayer.setReversed(true);
    haloStbdLayer.setBorder(false);

    allLEDsLayer.setMode(Mode.WHOLESTRIP);
    allLEDsLayer.setType(LayerType.SOLID);
    allLEDsLayer.setPriority(-9);
    allLEDsLayer.setColor(Color.kRed, Color.kBlack);
    allLEDsLayer.setBorder(false);


    io_Lights.addLayer(portStatusLayer);
    io_Lights.addLayer(stbdStatusLayer);
    io_Lights.addLayer(haloPortLayer);
    io_Lights.addLayer(haloStbdLayer);
    io_Lights.addLayer(allLEDsLayer);

    Triggers.bargeLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(5, true);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(5, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(5, false);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(5, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    }));
    Triggers.Lvl4LEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(5, true);
    portStatusLayer.setColor(Color.kBlack,Color.kWhite);
    stbdStatusLayer.setStatus(5, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kWhite);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(5, false);
    portStatusLayer.setColor(Color.kBlack,Color.kWhite);
    stbdStatusLayer.setStatus(5, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kWhite);
    }));
    Triggers.lvl3AlgaeLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(4, true);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(4, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(4, false);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(4, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    }));
    Triggers.lvl3CoralLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(4, true);
    portStatusLayer.setColor(Color.kBlack,Color.kWhite);
    stbdStatusLayer.setStatus(4, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kWhite);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(4, false);
    portStatusLayer.setColor(Color.kBlack,Color.kWhite);
    stbdStatusLayer.setStatus(4, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kWhite);
    }));
    Triggers.coralStationClawLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(3, true);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(3, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(3, false);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(3, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    }));
    Triggers.coralStationIntakeLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(3, true);
    portStatusLayer.setColor(Color.kBlack,Color.kWhite);
    stbdStatusLayer.setStatus(3, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kWhite);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(3, false);
    portStatusLayer.setColor(Color.kBlack,Color.kWhite);
    stbdStatusLayer.setStatus(3, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kWhite);
    }));
    Triggers.lvl2AlgaeLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(2, true);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(2, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(2, false);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(2, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    }));
    Triggers.lvl2CoralLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(2, true);
    portStatusLayer.setColor(Color.kBlack,Color.kWhite);
    stbdStatusLayer.setStatus(2, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kWhite);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(2, false);
    portStatusLayer.setColor(Color.kBlack,Color.kWhite);
    stbdStatusLayer.setStatus(2, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kWhite);
    }));
    Triggers.lvl1ClawLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(1, true);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(1, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(1, false);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(1, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    }));
    Triggers.lvl1CoralLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(1, true);
    portStatusLayer.setColor(Color.kBlack,Color.kWhite);
    stbdStatusLayer.setStatus(1, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kWhite);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(1, false);
    portStatusLayer.setColor(Color.kBlack,Color.kWhite);
    stbdStatusLayer.setStatus(1, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kWhite);
    }));
    Triggers.groundIntakeOrProcessorLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(0, true);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(0, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(0, false);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(0, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    }));
    Triggers.ClimbLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(0, true);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(0, true);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(0, false);
    portStatusLayer.setColor(Color.kBlack,Color.kTeal);
    stbdStatusLayer.setStatus(0, false);
    stbdStatusLayer.setColor(Color.kBlack,Color.kTeal);
    }));
    Triggers.stowedLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(1, true);
    stbdStatusLayer.setStatus(1, true);
    portStatusLayer.setStatus(3, true);
    stbdStatusLayer.setStatus(3, true);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setStatus(1, false);
    stbdStatusLayer.setStatus(1, false);
    portStatusLayer.setStatus(3, false);
    stbdStatusLayer.setStatus(3, false);
    }));
    Triggers.manualControlLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setColor(Color.kBlack,Color.kPurple);
    stbdStatusLayer.setColor(Color.kBlack,Color.kPurple);
    }));


    Triggers.eStopLEDs.onTrue(Commands.runOnce(
    () -> {
    portStatusLayer.setType(LayerType.ALTERNATING);
    stbdStatusLayer.setType(LayerType.ALTERNATING);
    })).onFalse(Commands.runOnce(
    () -> {
    portStatusLayer.setType(LayerType.STATUS);
    stbdStatusLayer.setType(LayerType.STATUS);
    }));
    Triggers.manualDriveLEDs.onTrue(Commands.runOnce(
    () -> {
    haloPortLayer.setType(LayerType.SOLID);
    haloPortLayer.setColor(Color.kRed,Color.kBlack);
    haloStbdLayer.setType(LayerType.SOLID);
    haloStbdLayer.setColor(Color.kRed,Color.kBlack);
    }));
    Triggers.headingLockLEDs.onTrue(Commands.runOnce(
    () -> {
    haloPortLayer.setType(LayerType.SOLID);
    haloPortLayer.setColor(Color.kOrange,Color.kBlack);
    haloStbdLayer.setType(LayerType.SOLID);
    haloStbdLayer.setColor(Color.kOrange,Color.kBlack);
    }));
    // Triggers.pathfindingLEDs.onTrue(Commands.runOnce(
    // () -> {
    // haloPortLayer.setType(LayerType.SCROLLER);
    // haloPortLayer.setColor(Color.kYellow,Color.kBlack);
    // haloStbdLayer.setType(LayerType.SCROLLER);
    // haloStbdLayer.setColor(Color.kYellow,Color.kBlack);
    // }));
    // Triggers.followPathLEDs.onTrue(Commands.runOnce(
    // () -> {
    // haloPortLayer.setType(LayerType.ALTERNATING);
    // haloPortLayer.setColor(Color.kYellow,Color.kBlack);
    // haloStbdLayer.setType(LayerType.ALTERNATING);
    // haloStbdLayer.setColor(Color.kYellow,Color.kBlack);
    // }));
    // Triggers.robotAtTargetLEDs.onTrue(Commands.runOnce(
    // () -> {
    // haloPortLayer.setType(LayerType.SOLID);
    // haloPortLayer.setColor(Color.kYellow,Color.kBlack);
    // haloStbdLayer.setType(LayerType.SOLID);
    // haloStbdLayer.setColor(Color.kYellow,Color.kBlack);
    // }));
    Triggers.robotArmAndClimberAtTargetLEDs.onTrue(Commands.runOnce(
    () -> {
    haloPortLayer.setType(LayerType.SOLID);
    haloPortLayer.setColor(Color.kGreen,Color.kBlack);
    haloStbdLayer.setType(LayerType.SOLID);
    haloStbdLayer.setColor(Color.kGreen,Color.kBlack);
    }));
    Triggers.atCoralStationLEDs.onTrue(Commands.runOnce(
    () -> {
    haloPortLayer.setType(LayerType.ALTERNATING);
    haloPortLayer.setColor(Color.kGreen,Color.kBlack);
    haloStbdLayer.setType(LayerType.ALTERNATING);
    haloStbdLayer.setColor(Color.kGreen,Color.kBlack);
    }));

//    allLEDsLayer.setPriority(-(allLEDsLayer.getPriority()));
  } 
  public Command getAutoCommand()
  {
    // Gets the input string of command phrases, processes into a list of commands, and puts them into a sequential command group
    return AutoUtils.getCommandList(SD.IO_AUTO.get(), s_Diffector, s_Coral, s_Algae);
  } 
}
