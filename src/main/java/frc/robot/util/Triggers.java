package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.HeadingStates;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DiffectorConstants.Presets;
import frc.robot.constants.FieldConstants;
import frc.robot.util.FieldUtils.GeoFencing;

public class Triggers 
{
  public static final Trigger unlockHeadingTrigger   = new Trigger(() -> Math.abs(RobotContainer.driver.getRawAxis(RobotContainer.rotationAxis)) > Constants.Control.stickDeadband);
  public static final Trigger cageDriveTrigger       = new Trigger(() -> RobotContainer.headingState == HeadingStates.CAGE_LOCK);
  public static final Trigger scoreDriveTrigger      = new Trigger(() -> RobotContainer.headingState == HeadingStates.REEF_LOCK);
  public static final Trigger stationDriveTrigger    = new Trigger(() -> RobotContainer.headingState == HeadingStates.STATION_LOCK);
  public static final Trigger processorDriveTrigger  = new Trigger(() -> RobotContainer.headingState == HeadingStates.PROCESSOR_LOCK);
  public static final Trigger autoScoreCancelTrigger = new Trigger
  (
    unlockHeadingTrigger.or
    (() -> 
      RobotContainer.driver.getRawAxis(RobotContainer.translationAxis) > Constants.Control.stickDeadband ||
      RobotContainer.copilot.getRawAxis(RobotContainer.manualDiffectorElevationAxis) > Constants.Control.manualDiffectorDeadband ||
      RobotContainer.copilot.getRawAxis(RobotContainer.manualDiffectorRotationAxis) > Constants.Control.manualDiffectorDeadband
    )
  );
  public static final Trigger driverLeftRumbleTrigger = new Trigger
  (
    () -> 
    (
      (FieldUtils.isRedAlliance() ? FieldUtils.GeoFencing.reefBlue : FieldUtils.GeoFencing.reefRed)
      .getCentre()
      .getDistance(RobotContainer.swerveState.Pose.getTranslation())
    ) 
    < 
    (FieldUtils.GeoFencing.circumscribedReefZoneDiameter / 2) + 1
  );
  public static final Trigger copilotLeftRumbleTrigger = new Trigger
  (
    () -> 
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coralIntakePositions = List.of(Presets.coralIntakePortPosition, Presets.coralIntakeStbdPosition).stream();
      var clawIntakePositions = List.of(Presets.coralClawPortPosition, Presets.coralClawStbdPosition).stream();
      var reefIntakePositions = List.of(Presets.algae2PortPosition, Presets.algae2StbdPosition, 
                                        Presets.algae3PortPosition, Presets.algae3StbdPosition).stream();
      return 
      (coralIntakePositions.anyMatch(position -> relativeTarget.equals(position)) && RobotContainer.coral) 
      || 
      (clawIntakePositions.anyMatch(position -> relativeTarget.equals(position)) && RobotContainer.algae)
      ||
      (reefIntakePositions.anyMatch(position -> relativeTarget.equals(position)) && RobotContainer.algae);
    }
  );
  public static final Trigger driverRightRumbleTrigger = new Trigger
  (
    () -> 
    {
      boolean northHalf = RobotContainer.swerveState.Pose.getTranslation().getX() >= FieldUtils.fieldWidth / 2;
      GeoFenceObject nearestCoralStation =
      FieldUtils.isRedAlliance() ?
      northHalf ? GeoFencing.cornerNRed : GeoFencing.cornerSRed
      :
      northHalf ? GeoFencing.cornerNBlue : GeoFencing.cornerSBlue;

      return 
      (RobotContainer.driver.rightBumper().getAsBoolean() && RobotContainer.algae)
      ||
      (copilotLeftRumbleTrigger.getAsBoolean() && nearestCoralStation.getDistance(RobotContainer.swerveState.Pose.getTranslation()) < FieldConstants.coralStationRange);
    }
  );
  public static final Trigger copliotRightRumbleTrigger = new Trigger
  (
    () -> 
    {
      Translation2d robotPos = RobotContainer.swerveState.Pose.getTranslation();
      Translation2d nearestClimbLineup = 
      FieldUtils.isRedAlliance() ? 
      robotPos.nearest(FieldConstants.redClimbLineups)
      :
      robotPos.nearest(FieldConstants.blueClimbLineups);

      return RobotContainer.s_Climber.climbReady() && RobotContainer.s_Diffector.climbReady() && RobotContainer.swerveState.Pose.getTranslation().getDistance(nearestClimbLineup) < Constants.Auto.atPosTolerance;
    }
  );
  public static final Trigger groundIntakeProcessorOrClimbLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var algaeIntakePositions = List.of(Presets.algaeIntakePortPosition, Presets.algaeIntakeStbdPosition).stream();
      var processorPositions = List.of(Presets.processorPositionPort, Presets.processorPositionStbd).stream();
      var climbPosition = List.of(Presets.climbPosition).stream();
      return
      algaeIntakePositions.anyMatch(position -> relativeTarget.equals(position))
      ||
      processorPositions.anyMatch(position -> relativeTarget.equals(position))
      ||
      climbPosition.anyMatch(position -> relativeTarget.equals(position));
    }
  );
  public static final Trigger lvl1LEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral1Positions = List.of(Presets.coral1PortPosition, Presets.coral1StbdPosition).stream();
      var claw1Positions = List.of(Presets.coral1ClawPortPosition, Presets.coral1ClawStbdPosition).stream();
      return
      (coral1Positions.anyMatch(position -> relativeTarget.equals(position)))
      ||
      (claw1Positions.anyMatch(position -> relativeTarget.equals(position)));
    }
  );
  public static final Trigger lvl2LEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral2Positions = List.of(Presets.coral2PortPosition, Presets.coral2StbdPosition).stream();
      var algae2Positions = List.of(Presets.algae2PortPosition, Presets.algae2StbdPosition).stream();
      return
      (coral2Positions.anyMatch(position -> relativeTarget.equals(position)))
      ||
      (algae2Positions.anyMatch(position -> relativeTarget.equals(position)));
    }
  );
  public static final Trigger coralStationLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coralIntakePositions = List.of(Presets.coralIntakePortPosition, Presets.coralIntakeStbdPosition).stream();
      var clawIntakePositions = List.of(Presets.coralClawPortPosition, Presets.coralClawStbdPosition).stream();

      return
      (coralIntakePositions.anyMatch(position -> relativeTarget.equals(position)))
      ||
      (clawIntakePositions.anyMatch(position -> relativeTarget.equals(position)));
    }
  );
  public static final Trigger lvl3LEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral3Positions = List.of(Presets.coral3PortPosition, Presets.coral3StbdPosition).stream();
      var algae3Positions = List.of(Presets.algae3PortPosition, Presets.algae3StbdPosition).stream();
      return
      (coral3Positions.anyMatch(position -> relativeTarget.equals(position)))
      ||
      (algae3Positions.anyMatch(position -> relativeTarget.equals(position)));
    }
  );
  public static final Trigger bargeOrLvl4LEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral4Positions = List.of(Presets.coral4PortPosition, Presets.coral4StbdPosition).stream();
      var algaeBargePosition = List.of(Presets.netPosition).stream();
      return
      (coral4Positions.anyMatch(position -> relativeTarget.equals(position)))
      ||
      (algaeBargePosition.anyMatch(position -> relativeTarget.equals(position)));
    }
  );

  public static final Trigger stowedLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coralStowPosition = List.of(Presets.coralStowPosition).stream();
      var algaeStowPosition = List.of(Presets.algaeStowPosition).stream();
      return
      (coralStowPosition.anyMatch(position -> relativeTarget.equals(position)))
      ||
      (algaeStowPosition.anyMatch(position -> relativeTarget.equals(position)));
    }
  );


  public static final Trigger manualDriveLEDs = new Trigger
  (
    () ->
    {
      return
    }
  );
  public static final Trigger headingLockLEDs = new Trigger
  (
    () ->
    {
      return
      cageDriveTrigger.getAsBoolean()
      ||
      scoreDriveTrigger.getAsBoolean()
      ||
      stationDriveTrigger.getAsBoolean()
      ||
      processorDriveTrigger.getAsBoolean();
    }
  );
  public static final Trigger pathfindingLEDs = new Trigger
  (
    () ->
    {
      return
    }
  );
  public static final Trigger followPathLEDs = new Trigger
  (
    () ->
    {
      return
    }
  );
  public static final Trigger robotAtTargetLEDs = new Trigger
  (
    () ->
    {
      return
    }
  );
  public static final Trigger robotArmAndClimberAtTargetLEDs = new Trigger
  (
    () ->
    {
      return
    }
  );
  public static final Trigger atCoralStationLEDs = new Trigger
  (
    () ->
    {
      return
    }
  );
}
