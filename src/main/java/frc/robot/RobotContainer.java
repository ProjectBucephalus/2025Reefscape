package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
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
import frc.robot.constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;
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
  public static final Intake s_Intake = new Intake();
  public static final CoralManipulator s_CoralManipulator = new CoralManipulator();
  public static final AlgaeManipulator s_AlgaeManipulator = new AlgaeManipulator();
  public static final CANifierAccess s_Canifier = new CANifierAccess();
  public static Rumbler s_Rumbler = new Rumbler(driver, copilot);

  /* Drive Controls */
  public static final int translationAxis = XboxController.Axis.kLeftY.value;
  public static final int strafeAxis = XboxController.Axis.kLeftX.value;
  public static final int rotationAxis = XboxController.Axis.kRightX.value;
  public static final int brakeAxis = XboxController.Axis.kRightTrigger.value;

  /* Triggers */
  public static final Trigger unlockHeadingTrigger = new Trigger(() -> Math.abs(driver.getRawAxis(rotationAxis)) > Constants.Control.stickDeadband);
  private final Trigger cageDriveTrigger = new Trigger(() -> headingState == HeadingStates.CAGE_LOCK);
  private final Trigger scoreDriveTrigger = new Trigger(() -> headingState == HeadingStates.REEF_LOCK);
  private final Trigger stationDriveTrigger = new Trigger(() -> headingState == HeadingStates.STATION_LOCK);
  private final Trigger processorDriveTrigger = new Trigger(() -> headingState == HeadingStates.PROCESSOR_LOCK);
  private final Trigger driverLeftRumbleTrigger = new Trigger(() -> s_Intake.getAlgaeState());
  private final Trigger copilotLeftRumbleTrigger = new Trigger(
              () -> s_Intake.getAlgaeState() && (s_Diffector.getRelativeRotation() > 45 && s_Diffector.getRelativeRotation() < 315) ||
              s_Intake.getAlgaeState() && (s_Diffector.getRelativeRotation() > 135 && s_Diffector.getRelativeRotation() < 225));
  //private final Trigger driverRightRumblTrigger = new Trigger(() -> )
  // TODO: Ready to score rumble
  //private final Trigger copliotRightRumbleTrigger = new Trigger(() -> s_Intake.climbReady() && s_Climber.climbReady() && s_Diffector.climbReady() );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    swerveState = s_Swerve.getState();

    SmartDashboard.putBoolean("IgnoreFence", true);
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

    s_Swerve.registerTelemetry(logger::telemeterize);
  }

  private void configureDriverBindings()
  {
    // Heading reset
    driver.start().onTrue(Commands.runOnce(() -> s_Swerve.getPigeon2().setYaw(RobotContainer.s_Swerve.getPigeon2().getYaw().getValueAsDouble() + (FieldUtils.isRedAlliance() ? 180 : 0))));

    /* Intake controls */
    driver.leftTrigger().whileTrue(new SetCoralStatus(s_CoralManipulator, CoralManipulatorStatus.DELIVERY_LEFT));
    driver.leftBumper().whileTrue(new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.PROCESSOR));

    /* Scoring and game piece management controls */
    driver.rightBumper().onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.INTAKE_ALGAE)).onFalse(new SetIntakeStatus(s_Intake, IntakeStatus.STOWED));
    //driver.back().onTrue(new AutoScoreSequence(s_Diffector, s_AlgaeManipulator, s_CoralManipulator, s_Swerve, () -> state.Pose.getTranslation()));
  }

  private void configureAutoDriveBindings()
  {
    /* Heading lock state management */
    unlockHeadingTrigger.onTrue(Commands.runOnce(() -> headingState = HeadingStates.UNLOCKED));
    driver.y().onTrue(Commands.runOnce(() -> headingState = HeadingStates.CAGE_LOCK));
    driver.a().onTrue(Commands.runOnce(() -> headingState = HeadingStates.STATION_LOCK));
    driver.b().onTrue(Commands.runOnce(() -> headingState = HeadingStates.PROCESSOR_LOCK));
    driver.x().onTrue(Commands.runOnce(() -> headingState = HeadingStates.REEF_LOCK));

    /* 
      * Cage pathfinding controls 
      * Drives to the nearest reef face when the cage heading lock is active and a corresponding dpad direction is pressed 
      */ 
    cageDriveTrigger.and(driver.povUp()).onTrue(new PathfindToAndFollow("cage2", s_Swerve));
    cageDriveTrigger.and(driver.povLeft()).onTrue(new PathfindToAndFollow("cage3", s_Swerve));
    cageDriveTrigger.and(driver.povRight()).onTrue(new PathfindToAndFollow("cage1", s_Swerve));

    /* 
      * Station pathfinding controls 
      * Drives to the nearest coral station when the station heading lock is active and a corresponding dpad direction is pressed 
      */ 
    stationDriveTrigger.and(driver.povUp()).onTrue(new PathfindToStation(5, () -> swerveState.Pose.getY(), s_Swerve));
    stationDriveTrigger.and(driver.povLeft()).onTrue(new PathfindToStation(2, () -> swerveState.Pose.getY(), s_Swerve));
    stationDriveTrigger.and(driver.povRight()).onTrue(new PathfindToStation(8, () -> swerveState.Pose.getY(), s_Swerve));

    /* 
      * Processor pathfinding control 
      * Runs when the processor heading lock is active and right is pressed on the dpad 
      */ 
    processorDriveTrigger.and(driver.povRight()).onTrue(new PathfindToAndFollow("p", s_Swerve));

    /* 
      * Reef pathfinding controls 
      * Drives to the nearest reef face when the reef heading lock is active and a corresponding dpad direction is pressed 
      */ 
    scoreDriveTrigger.and(driver.povUp()).onTrue(new PathfindToReef(DpadOptions.CENTRE, () -> swerveState.Pose.getTranslation(), s_Swerve));
    scoreDriveTrigger.and(driver.povLeft()).onTrue(new PathfindToReef(DpadOptions.LEFT, () -> swerveState.Pose.getTranslation(), s_Swerve));
    scoreDriveTrigger.and(driver.povRight()).onTrue(new PathfindToReef(DpadOptions.RIGHT, () -> swerveState.Pose.getTranslation(), s_Swerve));

    /* 
      * Net pathfinding controls 
      * Drives to the nearest net position when the scoring heading lock is active and down is pressed on the dpad
      */ 
    scoreDriveTrigger.and(driver.povDown()).onTrue(new PathfindToBarge(() -> swerveState.Pose.getTranslation(), s_Swerve));

    /* 
      * Binds heading targetting commands to run while the appropriate trigger is active and the dpad isn't pressed
      * Does not need to check the rotation stick, as soon at the rotation stick is moved all drive triggers become false (see line 141)
      * Bind heading targeting commands to run while the appropriate head lock trigger is active and the dpad isn't pressed
      * Does not need to check the rotation stick, as soon as the rotation stick is moved all heading lock triggers become false 
      * (see line 141)
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
          Rotation2d.kCW_90deg,
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
        new TargetHeading
        (
          s_Swerve,
          Rotation2d.kCW_90deg, 
          Rotation2d.kCW_90deg,
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> driver.getRawAxis(brakeAxis),
          () -> !driver.leftStick().getAsBoolean()
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
    //copilot.start().and(copilot.back()).onTrue(new MoveTo(s_Diffector, Constants.DiffectorConstants.climbElevation, Constants.DiffectorConstants.climbAngle)); //Starts climber
    //copilot.back().onTrue(new Test("climber", "deploy"));                                                                                           //Deploys the climber

    /* Coral scoring controls */
    copilot.a().and(copilot.rightTrigger().negate()).onTrue(new GoToCoralScorePos(1, s_Diffector));   //L1 scoring
    copilot.b().and(copilot.rightTrigger().negate()).onTrue(new GoToCoralScorePos(2, s_Diffector));   //L2 scoring
    copilot.x().and(copilot.rightTrigger().negate()).onTrue(new GoToCoralScorePos(3, s_Diffector));   //L3 scoring
    copilot.y().and(copilot.rightTrigger().negate()).onTrue(new GoToCoralScorePos(4, s_Diffector));   //L4 scoring
        
    /* Algae scoring/intaking controls */
    copilot.a().and(copilot.rightTrigger()).onTrue(new MoveTo(s_Diffector, Constants.DiffectorConstants.processorPosition));           //Processor scoring
    copilot.b().and(copilot.rightTrigger()).onTrue(new GoToAlgaeIntakePos(true, s_Diffector));  //L2 pick up
    copilot.x().and(copilot.rightTrigger()).onTrue(new GoToAlgaeIntakePos(false, s_Diffector)); //L3 pick up
    copilot.y().and(copilot.rightTrigger()).onTrue(new MoveTo(s_Diffector, Constants.DiffectorConstants.netPosition));            //Net scoring

    /* Stow pos*/
    copilot.povUp().and(copilot.rightTrigger()).onTrue(new MoveTo(s_Diffector, Constants.DiffectorConstants.algaeStowPosition));  //algae stow
    copilot.povUp().and(copilot.rightTrigger().negate()).onTrue(new MoveTo(s_Diffector, Constants.DiffectorConstants.coralStowPosition)); //coral stow

    /* Transfer pos */
    copilot.povDown().and(copilot.rightTrigger()).onTrue(new MoveTo(s_Diffector, Constants.DiffectorConstants.algaeTransferPosition));                         //agae transer
    copilot.povDown().and(copilot.rightTrigger().negate()).onTrue(new MoveTo(s_Diffector, Constants.DiffectorConstants.coralTransferPosition));                //coral transfer

    /* Game piece intake controls */
    copilot.leftTrigger().and(copilot.rightTrigger()).onTrue(new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.INTAKE_REEF));          //Intake algae through manipulator
    copilot.leftTrigger().and(copilot.rightTrigger().negate()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.TESTING)).onTrue(Commands.runOnce(() -> s_Intake.setAlgaeIntakeSpeed(Constants.IntakeConstants.algaeIntakeMotorSpeed), s_Intake)).onFalse(Commands.runOnce(() -> s_Intake.setAlgaeIntakeSpeed(0), s_Intake)); //intake algae from intake

    /* Game piece outtake controls */
    copilot.leftBumper().and(copilot.rightTrigger()).onTrue(new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.PROCESSOR)); //Ejects algae from manipulator
    copilot.leftBumper().and(copilot.rightTrigger().negate()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.TESTING)).onTrue(Commands.runOnce(() -> s_Intake.setAlgaeIntakeSpeed(Constants.IntakeConstants.algaeEjectMotorSpeed), s_Intake)).onFalse(Commands.runOnce(() -> s_Intake.setAlgaeIntakeSpeed(0), s_Intake)); //Ejects algae from intake

    /* Modifier controls for testing only */
    //copilot.rightTrigger().onTrue(new Test("algaeModifier", "on")).onFalse(new Test("algaeModifier", "off"));
    copilot.rightBumper().and(copilot.rightTrigger().negate()).onTrue(new MoveTo(s_Diffector, Constants.DiffectorConstants.coralIntakePosition));
    copilot.rightBumper().and(copilot.rightTrigger()).onTrue(new MoveTo(s_Diffector, Constants.DiffectorConstants.algaeIntakePosition));
    
    /* Manual intake controls */
    copilot.axisLessThan(XboxController.Axis.kLeftY.value, -Constants.Control.stickDeadband).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.TESTING)).whileTrue(Commands.run(() -> s_Intake.setAlgaeArmTarget(s_Intake.getAlgaeArmTarget() - 5), s_Intake));                      //Algae intake arm down
    copilot.axisGreaterThan(XboxController.Axis.kLeftY.value, Constants.Control.stickDeadband).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.TESTING)).whileTrue(Commands.run(() -> s_Intake.setAlgaeArmTarget(s_Intake.getAlgaeArmTarget() + 5), s_Intake));          // Algae intake arm up

    /* Manual arm controls */
    copilot.axisGreaterThan(XboxController.Axis.kRightX.value, Constants.Control.stickDeadband).whileTrue(new ManualArmControl(s_Diffector, () -> 5)); //Arm clockwise
    copilot.axisLessThan(XboxController.Axis.kRightX.value, -Constants.Control.stickDeadband).whileTrue(new ManualArmControl(s_Diffector, () -> -5));             //Arm anticlockwise
  
    /* Manual elevator controls */
    copilot.axisLessThan(XboxController.Axis.kRightY.value, -Constants.Control.stickDeadband).whileTrue(new ManualElevatorControl(s_Diffector, () -> -0.05));             //Elevator up
    copilot.axisGreaterThan(XboxController.Axis.kRightY.value, Constants.Control.stickDeadband).whileTrue(new ManualElevatorControl(s_Diffector, () -> 0.05)); //Elevator down
  }

  private void configureTestBindings()
  {
    testing.povUp().whileTrue(new ManualElevatorControl(s_Diffector, () -> 0.1));
    testing.povDown().whileTrue(new ManualElevatorControl(s_Diffector, () -> -0.1));

    testing.povLeft().whileTrue(new ManualArmControl(s_Diffector, () -> 15));
    testing.povRight().whileTrue(new ManualArmControl(s_Diffector, () -> -15));

    testing.rightBumper().onTrue(Commands.runOnce(() -> s_Diffector.setElevationTarget(Constants.DiffectorConstants.maxZ - 0.25), s_Diffector));
    testing.leftBumper().onTrue(Commands.runOnce(() -> s_Diffector.setElevationTarget(Constants.DiffectorConstants.startPosition.getX()), s_Diffector));

    testing.x().onTrue(Commands.runOnce(() -> s_AlgaeManipulator.setAlgaeManipulatorStatus(AlgaeManipulatorStatus.INTAKE_REEF)));
    testing.y().onTrue(Commands.runOnce(() -> s_AlgaeManipulator.setAlgaeManipulatorStatus(AlgaeManipulatorStatus.NET))).onFalse(Commands.runOnce(() -> s_AlgaeManipulator.setAlgaeManipulatorStatus(AlgaeManipulatorStatus.EMPTY)));
    testing.a().onTrue(Commands.runOnce(() -> s_CoralManipulator.setCoralManipulatorStatus(CoralManipulatorStatus.DELIVERY_LEFT))).onFalse(Commands.runOnce(() -> s_CoralManipulator.setCoralManipulatorStatus(CoralManipulatorStatus.DEFAULT)));
  }

  private void configureRumbleBindings()
  {
    /* Driver rumble bindings */
    
    driverLeftRumbleTrigger.onTrue(new SetRumble(s_Rumbler, Sides.DRIVER_LEFT, "Intake Full"));
    
    /* Copilot rumble bindings */
    copilotLeftRumbleTrigger.onTrue(new SetRumble(s_Rumbler, Sides.COPILOT_LEFT, "Transfer Ready"));
    //copliotRightRumbleTrigger.onTrue(new SetRumble(s_Rumbler, Sides.COPILOT_RIGHT, "Climb Ready"));
  }

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
    return new RunAutoCommandList(DynamicAuto.getCommandList(SmartDashboard.getString("Auto Input", Constants.Auto.defaultAuto)));
  }
}
