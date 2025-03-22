package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.HeadingStates;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DiffectorConstants.Presets;
import frc.robot.constants.FieldConstants;

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
  public static final Trigger opposingBargeZoneTrigger = new Trigger
  (
    () -> 
    (
      (FieldUtils.isRedAlliance() ? FieldUtils.GeoFencing.bargeZoneBlue : FieldUtils.GeoFencing.bargeZoneRed)
      .getDistance(RobotContainer.swerveState.Pose.getTranslation())
    ) 
    < FieldConstants.bargeWarningRange
  );
  public static final Trigger opposingReefZoneTrigger = new Trigger
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
  public static final Trigger homeReefZoneTrigger = new Trigger
  (
    () -> 
    (
      (FieldUtils.isRedAlliance() ? FieldUtils.GeoFencing.reefRed : FieldUtils.GeoFencing.reefBlue)
      .getCentre()
      .getDistance(RobotContainer.swerveState.Pose.getTranslation())
    ) 
    < 
    (FieldUtils.GeoFencing.circumscribedReefZoneDiameter / 2) + 0.3
  );
  public static final Trigger algaeIntakePosTrigger = new Trigger
  (
    homeReefZoneTrigger.and
    (
      () ->
      {
        Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
        var reefIntakePositions = List.of(Presets.algae2PortPosition, Presets.algae2StbdPosition, Presets.algae3PortPosition, Presets.algae3StbdPosition).stream();
        return (reefIntakePositions.anyMatch(relativeTarget::equals));
      }
    )
  );
  public static final Trigger algaeIntakeTrigger = algaeIntakePosTrigger.and(() -> !RobotContainer.algae);
  public static final Trigger coralIntakeTrigger = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coralIntakePositions = List.of(Presets.coralIntakePortPosition, Presets.coralIntakeStbdPosition).stream();
      var clawIntakePositions = List.of(Presets.coralClawPortPosition, Presets.coralClawStbdPosition).stream();
      return 
      (coralIntakePositions.anyMatch(relativeTarget::equals) && RobotContainer.coral) 
      || 
      (clawIntakePositions.anyMatch(relativeTarget::equals) && RobotContainer.algae);
    }
  );
  public static final Trigger atCoralStationTrigger = new Trigger
  (
    () -> 
    {
      Translation2d robotPos = RobotContainer.swerveState.Pose.getTranslation();
      return FieldUtils.getNearestCoralStation(robotPos).getDistance(robotPos) < FieldConstants.coralStationRange;
    }
  );
  public static final Trigger copilotLeftRumbleTrigger = coralIntakeTrigger.or(algaeIntakePosTrigger.and(() -> RobotContainer.algae));
  public static final Trigger driverRightRumbleTrigger = 
    coralIntakeTrigger
    .and(atCoralStationTrigger)
    .or
    (
      () ->
      {
        var groundIntakePositions = List.of(Presets.algaeIntakePortPosition, Presets.algaeIntakeStbdPosition).stream();
        return groundIntakePositions.anyMatch(position -> RobotContainer.s_Diffector.getRelativeTarget().equals(position)) && RobotContainer.algae;
      }
    )
    .or(homeReefZoneTrigger.and(() -> RobotContainer.algae));
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
  public static final Trigger bargeLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var algaeBargePosition = List.of(Presets.netPosition).stream();
      return
      (algaeBargePosition.anyMatch(relativeTarget::equals));
    }
  );
  public static final Trigger Lvl4LEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral4Positions = List.of(Presets.coral4PortPosition, Presets.coral4StbdPosition).stream();
      return
      (coral4Positions.anyMatch(relativeTarget::equals));
    }
  );
  public static final Trigger lvl3AlgaeLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var algae3Positions = List.of(Presets.algae3PortPosition, Presets.algae3StbdPosition).stream();
      return
      algae3Positions.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger lvl3CoralLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral3Positions = List.of(Presets.coral3PortPosition, Presets.coral3StbdPosition).stream();
      return
      coral3Positions.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger coralStationClawLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var clawIntakePositions = List.of(Presets.coralClawPortPosition, Presets.coralClawStbdPosition).stream();
      return
      clawIntakePositions.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger coralStationIntakeLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coralIntakePositions = List.of(Presets.coralIntakePortPosition, Presets.coralIntakeStbdPosition).stream();
      return
      coralIntakePositions.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger lvl2AlgaeLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var algae2Positions = List.of(Presets.algae2PortPosition, Presets.algae2StbdPosition).stream();
      return
      algae2Positions.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger lvl2CoralLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral2Positions = List.of(Presets.coral2PortPosition, Presets.coral2StbdPosition).stream();
      return
      coral2Positions.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger lvl1ClawLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var claw1Positions = List.of(Presets.coral1ClawPortPosition, Presets.coral1ClawStbdPosition).stream();
      return
      claw1Positions.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger lvl1CoralLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral1Positions = List.of(Presets.coral1PortPosition, Presets.coral1StbdPosition).stream();
      return
      coral1Positions.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger groundIntakeOrProcessorLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var algaeLowPositions = List.of(Presets.algaeIntakePortPosition, Presets.algaeIntakeStbdPosition, Presets.processorPositionPort, Presets.processorPositionStbd).stream();
      return
      algaeLowPositions.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger ClimbLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var climbPosition = List.of(Presets.climbPosition).stream();
      return
      climbPosition.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger stowedLEDs = new Trigger
  (
    () ->
    {
      Translation2d relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var stowPositions = List.of(Presets.coralStowPosition, Presets.algaeStowPosition).stream();
      return
      stowPositions.anyMatch(relativeTarget::equals);
    }
  );
  public static final Trigger manualControlLEDs = new Trigger(
    () ->
    {
      return
      RobotContainer.copilot.axisGreaterThan(5,0.8).getAsBoolean() ||
      RobotContainer.copilot.axisGreaterThan(5,-0.8).getAsBoolean() ||
      RobotContainer.copilot.axisGreaterThan(6,-0.8).getAsBoolean() ||
      RobotContainer.copilot.axisGreaterThan(6,0.8).getAsBoolean();
    }
  );
  

  public static final Trigger eStopLEDs = new Trigger(
    () ->
    {
      return
      SD.DIFF_ESTOP.get();
    }
  );
  public static final Trigger manualDriveLEDs = new Trigger
  (
    () ->
    {
      return unlockHeadingTrigger.getAsBoolean();
    }
  );
  public static final Trigger headingLockLEDs = new Trigger
  (
    () ->
    {
      return
      cageDriveTrigger.getAsBoolean() ||
      scoreDriveTrigger.getAsBoolean() ||
      stationDriveTrigger.getAsBoolean() ||
      processorDriveTrigger.getAsBoolean();
    }
  );
  // public static final Trigger pathfindingLEDs = new Trigger
  // (
  //   () ->
  //   {
  //     return
  //   }
  // );
  // public static final Trigger followPathLEDs = new Trigger
  // (
  //   () ->
  //   {
  //     Translation2d robotPos = RobotContainer.swerveState.Pose.getTranslation();
  //     return
  //   }
  // );
  // public static final Trigger robotAtTargetLEDs = new Trigger
  // (
  //   () ->
  //   {
  //     Translation2d robotPos = RobotContainer.swerveState.Pose.getTranslation();
  //     return
      
  //   }
  // );
  public static final Trigger robotArmAndClimberAtTargetLEDs = new Trigger
  (
    () ->
    {
      Translation2d robotPos = RobotContainer.swerveState.Pose.getTranslation();
      Translation2d nearestClimbLineup = 
      FieldUtils.isRedAlliance() ? 
      robotPos.nearest(FieldConstants.redClimbLineups)
      :
      robotPos.nearest(FieldConstants.blueClimbLineups);
      return
      RobotContainer.s_Climber.climbReady() && RobotContainer.s_Diffector.climbReady() && RobotContainer.swerveState.Pose.getTranslation().getDistance(nearestClimbLineup) < Constants.Auto.atPosTolerance;
    }
  );
  public static final Trigger atCoralStationLEDs = atCoralStationTrigger;
}
