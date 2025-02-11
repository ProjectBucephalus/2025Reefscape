package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  /* Persistent values for tracking systems */
  public static HeadingStates headingState = HeadingStates.UNLOCKED;
  public static boolean coral = Constants.Diffector.startingCoralState;
  public static boolean algae = Constants.Diffector.startingAlgaeState;

  /* Controllers */
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController copilot = new CommandXboxController(1);
  public static final CommandXboxController testing = new CommandXboxController(3);

  /* Subsystems */
  public static final CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
  public static final Limelight s_LimelightPort = new Limelight("limelight-port");
  public static final Limelight s_LimelightStbd = new Limelight("limelight-stbd");
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
  private final Trigger driverLeftRumbleTrigger = new Trigger(() -> s_Intake.getCoralState() || s_Intake.getAlgaeState());
  private final Trigger copilotLeftRumbleTrigger = new Trigger(
              () -> s_Intake.isCoralStowed() ||s_Intake.getCoralState() ||s_Intake.getAlgaeState() 
              || s_Intake.getCoralState() && (s_Diffector.getEncoderPos() > 45 && s_Diffector.getEncoderPos() < 315) ||
              s_Intake.getAlgaeState() && (s_Diffector.getEncoderPos() > 135 && s_Diffector.getEncoderPos() < 225));
  //private final Trigger driverRightRumblTrigger = new Trigger(() -> )
  // TODO: Ready to score rumble
  private final Trigger copliotRightRumbleTrigger = new Trigger(() -> s_Intake.climbReady() && s_Climber.climbReady() && s_Diffector.climbReady() );

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

    // Configure button bindings
    configureDriverBindings();
    configureAutoDriveBindings();
    configureCopilotBindings();
    configureTestBindings();
    configureRumbleBindings();
  }

  private void configureDriverBindings()
  {
    // Heading reset
    driver.start().onTrue(Commands.runOnce(() -> s_Swerve.seedFieldCentric()));

    /* Intake controls */
    driver.leftTrigger().whileTrue(Commands.runOnce(() -> s_Intake.setIntakeStatus(IntakeStatus.INTAKE_CORAL)));
    driver.leftBumper().whileTrue(Commands.runOnce(() -> s_Intake.setIntakeStatus(IntakeStatus.INTAKE_ALGAE)));

    /* Scoring and game piece management controls */
    driver.rightBumper().whileTrue(new DropGamePiece(s_AlgaeManipulator, s_CoralManipulator));
    driver.back().onTrue(new AutoScoreSequence(s_Diffector, s_AlgaeManipulator, s_CoralManipulator, s_Swerve, () -> s_Swerve.getState().Pose.getTranslation()));
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
    stationDriveTrigger.and(driver.povUp()).onTrue(new PathfindToStation(5, () -> s_Swerve.getState().Pose.getY(), s_Swerve));
    stationDriveTrigger.and(driver.povLeft()).onTrue(new PathfindToStation(2, () -> s_Swerve.getState().Pose.getY(), s_Swerve));
    stationDriveTrigger.and(driver.povRight()).onTrue(new PathfindToStation(8, () -> s_Swerve.getState().Pose.getY(), s_Swerve));

    /* 
      * Processor pathfinding control 
      * Runs when the processor heading lock is active and right is pressed on the dpad 
      */ 
    processorDriveTrigger.and(driver.povRight()).onTrue(new PathfindToAndFollow("p", s_Swerve));

    /* 
      * Reef pathfinding controls 
      * Drives to the nearest reef face when the reef heading lock is active and a corresponding dpad direction is pressed 
      */ 
    reefDriveTrigger.and(driver.povUp()).onTrue(new PathfindToReef(DpadOptions.CENTRE, () -> s_Swerve.getState().Pose.getTranslation(), s_Swerve));
    reefDriveTrigger.and(driver.povLeft()).onTrue(new PathfindToReef(DpadOptions.LEFT, () -> s_Swerve.getState().Pose.getTranslation(), s_Swerve));
    reefDriveTrigger.and(driver.povRight()).onTrue(new PathfindToReef(DpadOptions.RIGHT, () -> s_Swerve.getState().Pose.getTranslation(), s_Swerve));

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
  }

  private void configureCopilotBindings()
  {
    /* Climb controls */
    copilot.start().and(copilot.back()).onTrue(new MoveTo(s_Diffector, Constants.Diffector.climbElevation, Constants.Diffector.climbAngle));
    copilot.back().onTrue(new Test("climber", "deploy"));

    /* Coral scoring controls */
    copilot.a().and(copilot.rightTrigger().negate()).onTrue(new ScoreCoralSequence(1, s_Diffector, s_CoralManipulator));
    copilot.b().and(copilot.rightTrigger().negate()).onTrue(new ScoreCoralSequence(2, s_Diffector, s_CoralManipulator));
    copilot.x().and(copilot.rightTrigger().negate()).onTrue(new ScoreCoralSequence(3, s_Diffector, s_CoralManipulator));
    copilot.y().and(copilot.rightTrigger().negate()).onTrue(new ScoreCoralSequence(4, s_Diffector, s_CoralManipulator));
        
    /* Algae scoring controls */
    copilot.a().and(copilot.rightTrigger()).onTrue(new ScoreAlgae(false, s_Diffector, s_AlgaeManipulator));
    copilot.b().and(copilot.rightTrigger()).onTrue(new IntakeAlgaeSequence(true, s_Diffector, s_AlgaeManipulator));
    copilot.x().and(copilot.rightTrigger()).onTrue(new IntakeAlgaeSequence(false, s_Diffector, s_AlgaeManipulator));
    copilot.y().and(copilot.rightTrigger()).onTrue(new ScoreAlgae(true, s_Diffector, s_AlgaeManipulator));

    /* Game piece transfer positions controls*/
    copilot.povUp().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.TRANSFER_ALGAE));
    copilot.povUp().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new MoveTo(s_Diffector, Constants.Diffector.algaeTransferElevation, Constants.Diffector.algaeTransferAngle));
    copilot.povUp().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.TRANSFER_ALGAE));
    copilot.povUp().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new MoveTo(s_Diffector, Constants.Diffector.coralTransferElevation, Constants.Diffector.coralTransferAngle));

    /* Transfer and deploy controls */
    copilot.povDown().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.STAND_BY));
    copilot.povDown().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new TransferGamePiece(s_Diffector, s_Intake, false));
    copilot.povDown().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.STAND_BY));
    copilot.povDown().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new TransferGamePiece(s_Diffector, s_Intake, true));

    /* Stow position controls */
    copilot.povRight().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.STOWED));
    copilot.povRight().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new IntakeAlgae(s_AlgaeManipulator));
    copilot.povRight().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.STOWED));
    copilot.povRight().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new IntakeCoral(s_CoralManipulator));

    // Intake from coral station
    copilot.povLeft().and(copilot.rightTrigger().negate()).whileTrue(new IntakeCoralSequence(s_Diffector, s_CoralManipulator));

    /* Game piece intake controls */
    copilot.leftTrigger().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.INTAKE_ALGAE));
    copilot.leftTrigger().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new IntakeAlgae(s_AlgaeManipulator));
    copilot.leftTrigger().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.INTAKE_CORAL));
    copilot.leftTrigger().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new IntakeCoral(s_CoralManipulator));

    /* Game piece outtake controls */
    copilot.leftBumper().and(copilot.rightBumper()).and(copilot.rightTrigger()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.EJECT_ALGAE));
    copilot.leftBumper().and(copilot.rightBumper().negate()).and(copilot.rightTrigger()).onTrue(new EjectAlgae(s_AlgaeManipulator));
    copilot.leftBumper().and(copilot.rightBumper()).and(copilot.rightTrigger().negate()).onTrue(new SetIntakeStatus(s_Intake, IntakeStatus.EJECT_CORAL));
    copilot.leftBumper().and(copilot.rightBumper().negate()).and(copilot.rightTrigger().negate()).onTrue(new EjectCoral(s_CoralManipulator));

    /* Modifier controls for testing only */
    copilot.rightTrigger().onTrue(new Test("algaeModifier", "on")).onFalse(new Test("algaeModifier", "off"));
    copilot.rightBumper().onTrue(new Test("intakeModifier", "on")).onFalse(new Test("intakeModifier", "off"));
    
    /* Manual winch controls */
    copilot.axisGreaterThan(0, 0.85).and(copilot.back()).whileTrue(new Test("manualWinch", "spool winch"));
    copilot.axisLessThan(0, -0.85).and(copilot.back()).whileTrue(new Test("manualWinch", "unspool winch"));
    
    /* Manual intake controls */
    copilot.axisLessThan(1, -0.85).and(copilot.rightTrigger()).onTrue(new Test("manualIntake", "intake up"));
    copilot.axisGreaterThan(1, 0.85).and(copilot.rightTrigger()).onTrue(new Test("manualIntake", "intake down"));

    /* Manual arm controls */
    copilot.axisGreaterThan(4, 0.85).onTrue(new Test("manualArm", "arm clockwise"));
    copilot.axisLessThan(4, -0.85).onTrue(new Test("manualArm", "arm anticlockwise"));
  
    /* Manual elevator controls */
    copilot.axisLessThan(5, -0.85).whileTrue(new Test("manualElevator", "elevator up"));
    copilot.axisGreaterThan(5, 0.85).whileTrue(new Test("manualElevator", "elevator down"));
  }

  private void configureTestBindings()
  {
    testing.y().onTrue(new InstantCommand(() -> s_Diffector.testingOveride(true, 0.2))).onFalse(new InstantCommand(() -> s_Diffector.testingOveride(true, 0)));
    testing.a().onTrue(new InstantCommand(() -> s_Diffector.testingOveride(true, -0.2))).onFalse(new InstantCommand(() -> s_Diffector.testingOveride(true, 0)));
    testing.povUp().onTrue(new InstantCommand(() -> s_Diffector.testingOveride(false, -0.2))).onFalse(new InstantCommand(() -> s_Diffector.testingOveride(false, 0)));
    testing.povDown().onTrue(new InstantCommand(() -> s_Diffector.testingOveride(false, 0.2))).onFalse(new InstantCommand(() -> s_Diffector.testingOveride(false, 0)));
  }

  private void configureRumbleBindings()
  {
    /* Driver rumble bindings */
    driverLeftRumbleTrigger.onTrue(new SetRumble(s_Rumbler, Sides.DRIVER_LEFT, "Intake Full"));
    
    /* Copilot rumble bindings */
    copilotLeftRumbleTrigger.onTrue(new SetRumble(s_Rumbler, Sides.COPILOT_LEFT, "Transfer Ready"));
    copliotRightRumbleTrigger.onTrue(new SetRumble(s_Rumbler, Sides.COPILOT_RIGHT, "Climb Ready"));
  }

  public Command getAutoCommand()
  {
    // Gets the input string of command phrases, processes into a list of commands, and puts them into a sequential command group
    return new RunAutoCommandList(DynamicAuto.getCommandList(SmartDashboard.getString("Auto Input", Constants.Auto.defaultAuto)));
  }
}
