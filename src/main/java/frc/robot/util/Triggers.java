package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.HeadingStates;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DiffectorConstants.Presets;
import frc.robot.constants.FieldConstants;

public class Triggers 
{
  public static final Trigger unlockHeadingTrigger   = new Trigger(() -> RobotContainer.headingState == HeadingStates.UNLOCKED || Math.abs(RobotContainer.driver.getRawAxis(RobotContainer.rotationAxis)) > Constants.Control.stickDeadband);
  public static final Trigger cageDriveTrigger       = new Trigger(() -> RobotContainer.headingState == HeadingStates.CAGE_LOCK);
  public static final Trigger scoreDriveTrigger      = new Trigger(() -> RobotContainer.headingState == HeadingStates.REEF_LOCK);
  public static final Trigger stationDriveTrigger    = new Trigger(() -> RobotContainer.headingState == HeadingStates.STATION_LOCK);
  public static final Trigger processorDriveTrigger  = new Trigger(() -> RobotContainer.headingState == HeadingStates.PROCESSOR_LOCK);
  public static final Trigger allowAutoDriveTrigger  = new Trigger(() -> SD.IO_LL.get());
  public static final Trigger demoAllowAutoDriveTrigger = allowAutoDriveTrigger.and(() -> !SD.STATE_DEMO.get());
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
  public static final Trigger usePenaltyRumbleTrigger = new Trigger
  (
    () -> SD.IO_LL.get() && SD.IO_GEOFENCE.get()
  );
  public static final Trigger algaeIntakePosTrigger = new Trigger
  (
    homeReefZoneTrigger.and
    (
      () ->
      {
        ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
        var reefIntakePositions = List.of(Presets.algae2Position, Presets.algae3Position).stream();
        return (reefIntakePositions.anyMatch(relativeTarget::relativeEquals));
      }
    )
  );
  public static final Trigger algaeIntakeTrigger = algaeIntakePosTrigger.and(() -> !RobotContainer.algae);
  public static final Trigger coralIntakeTrigger = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coralIntakePositions = List.of(Presets.coralIntakePosition).stream();
      var clawIntakePositions = List.of(Presets.coralClawPosition).stream();
      return 
      (coralIntakePositions.anyMatch(relativeTarget::relativeEquals) && RobotContainer.coral) 
      || 
      (clawIntakePositions.anyMatch(relativeTarget::relativeEquals) && RobotContainer.algae);
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
        var groundIntakePositions = List.of(Presets.algaeIntakePosition).stream();
        return groundIntakePositions.anyMatch(position -> RobotContainer.s_Diffector.getRelativeTarget().relativeEquals(position)) && RobotContainer.algae;
      }
    )
    .or(homeReefZoneTrigger.and(() -> RobotContainer.algae));


  public static final Trigger bargeLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var algaeBargePosition = List.of(Presets.netPosition).stream();
      return
      (algaeBargePosition.anyMatch(relativeTarget::relativeEquals));
    }
  );
  public static final Trigger Lvl4LEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral4Positions = List.of(Presets.coral4Position).stream();
      return
      (coral4Positions.anyMatch(relativeTarget::relativeEquals));
    }
  );
  public static final Trigger lvl3AlgaeLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var algae3Positions = List.of(Presets.algae3Position).stream();
      return
      algae3Positions.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger lvl3CoralLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral3Positions = List.of(Presets.coral3Position, Presets.coral3AltPosition).stream();
      return
      coral3Positions.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger coralStationClawLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var clawIntakePositions = List.of(Presets.coralClawPosition).stream();
      return
      clawIntakePositions.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger coralStationIntakeLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coralIntakePositions = List.of(Presets.coralIntakePosition, Presets.coralIntakeAltPosition).stream();
      return
      coralIntakePositions.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger lvl2AlgaeLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var algae2Positions = List.of(Presets.algae2Position).stream();
      return
      algae2Positions.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger lvl2CoralLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral2Positions = List.of(Presets.coral2Position, Presets.coral2AltPosition).stream();
      return
      coral2Positions.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger lvl1ClawLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var claw1Positions = List.of(Presets.coral1ClawPosition).stream();
      return
      claw1Positions.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger lvl1CoralLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var coral1Positions = List.of(Presets.coral1Position).stream();
      return
      coral1Positions.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger groundIntakeOrProcessorLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var algaeLowPositions = List.of(Presets.algaeIntakePosition, Presets.processorPosition).stream();
      return
      algaeLowPositions.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger ClimbLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var climbPosition = List.of(Presets.climbPosition).stream();
      return
      climbPosition.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger stowedLEDs = new Trigger
  (
    () ->
    {
      ArmPos relativeTarget = RobotContainer.s_Diffector.getRelativeTarget();
      var stowPositions = List.of(Presets.coralStowPosition, Presets.algaeStowPosition).stream();
      return
      stowPositions.anyMatch(relativeTarget::relativeEquals);
    }
  );
  public static final Trigger manualControlLEDs = new Trigger
  (
    () ->
    {
      return
      RobotContainer.copilot.axisMagnitudeGreaterThan(5,0.3).getAsBoolean() ||
      RobotContainer.copilot.axisMagnitudeGreaterThan(6,0.3).getAsBoolean();
    }
  );
  

  public static final Trigger eStopLEDs = new Trigger
  (
    SD.DIFF_ESTOP::get
  );
    
  public static final Trigger manualDriveLEDs = unlockHeadingTrigger;
  
  public static final Trigger intakeFullLEDs = new Trigger
  (
    () -> {
      return atCoralStationTrigger.getAsBoolean() && (RobotContainer.coral || RobotContainer.algae);
    }
  );

  public static final Trigger headingLockLEDs = new Trigger
  (
    () -> SD.STATE_DRIVE.get().equals("Heading Locked")
  )
  .and(atCoralStationTrigger.negate());

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
        RobotContainer.s_Climber.climbReady() && 
        RobotContainer.s_Diffector.climbReady() && 
        RobotContainer.swerveState.Pose.getTranslation()
          .getDistance(nearestClimbLineup) < Constants.Auto.atPosTolerance;
    }
  );

  public static final Trigger atCoralStationLEDs = atCoralStationTrigger;

  public static final Trigger timerClimbLEDs = new Trigger
  (
    () -> {
      return (DriverStation.isTeleop() && MathUtil.isNear(Timer.getMatchTime(), SD.IO_CLIMB_WARNING.get(), 2));
    }
  );

  public static final Trigger pathFollowing = new Trigger
    (() -> {return SD.STATE_DRIVE.get().equals("Following");});

  public static final Trigger pathTarget = new Trigger
    (() -> {return SD.STATE_DRIVE.get().equals("At Target");});

  public static final Trigger armAtTarget = new Trigger
    (() -> RobotContainer.s_Diffector.atPosition());

  public static final Trigger armAtReefCoral = new Trigger
    (() -> {
      return 
      (
        RobotContainer.s_Diffector.atRelativePosition(Presets.coral4Position) ||
        RobotContainer.s_Diffector.atRelativePosition(Presets.coral3Position) ||
        RobotContainer.s_Diffector.atRelativePosition(Presets.coral2Position) ||
        RobotContainer.s_Diffector.atRelativePosition(Presets.coral1Position)
      );
    });

  public static final Trigger armAtReefAlgae = new Trigger
  (() -> {
    return 
    (
      RobotContainer.s_Diffector.atRelativePosition(Presets.algae3Position) ||
      RobotContainer.s_Diffector.atRelativePosition(Presets.algae2Position)
    );
  });
}
