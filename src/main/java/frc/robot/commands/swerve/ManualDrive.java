// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.*;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.SD;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualDrive extends SwerveCommandBase 
{
  protected DoubleSupplier rotationSup;
  protected double rotationVal;

  protected final SwerveRequest.FieldCentric driveRequest = new SwerveRequest
    .FieldCentric() 
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /** Creates a new ManualDrive. */
  public ManualDrive(CommandSwerveDrivetrain s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier brakeSup, BooleanSupplier fencedSup) 
  {
    super(s_Swerve, translationSup, strafeSup, brakeSup, fencedSup);
    this.rotationSup = rotationSup;
  }

  @Override
  public void execute()
  {
    processXY();

    /* Get and process Rotation input */
    rotationVal = rotationSup.getAsDouble();
    if (Math.abs(rotationVal) <= deadband) {rotationVal = 0;}
    rotationVal *= MathUtil.interpolate(Control.maxRotThrottle, Control.minRotThrottle, brakeVal);

    if (motionXY.getNorm() != 0)
      {SD.STATE_DRIVE.put("Manual");}

    s_Swerve.setControl
    (
      driveRequest
      .withVelocityX(motionXY.getX() * Swerve.maxSpeed)
      .withVelocityY(motionXY.getY() * Swerve.maxSpeed)
      .withRotationalRate(rotationVal * Swerve.maxAngularVelocity)
    );
  }
}
