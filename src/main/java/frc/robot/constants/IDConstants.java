// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix.CANifier.GeneralPin;

public final class IDConstants 
{   
  public static final int pdhID = 0;

  /* Drive */
  /* _____ */
  public static final int foreStbdDriveMotorID = 1;
  public static final int foreStbdAngleMotorID = 2;
  public static final int foreStbdCANcoderID   = 3;

  public static final int forePortDriveMotorID = 4;
  public static final int forePortAngleMotorID = 5;
  public static final int forePortCANcoderID   = 6;

  public static final int aftPortDriveMotorID = 7;
  public static final int aftPortAngleMotorID = 8;
  public static final int aftPortCANcoderID   = 9;

  public static final int aftStbdDriveMotorID = 10;
  public static final int aftStbdAngleMotorID = 11;
  public static final int aftStbdCANcoderID   = 12;

  public static final int pigeonID = 53; // Pigeon ID hardset to 53, we can't change it

  /* Diffector */
  /* _________ */
  public static final int armCANcoderID = 13;
  public static final int uaMotorID    = 14; // Port
  public static final int daMotorID    = 15; // Stbd

  /* Intake */
  /* ______ */
  public static final int algaeIntakeRollerID = 16;
  public static final int algaeIntakeArmID    = 18;

  public static final int coralIntakeDIOPort = 1;
  public static final int coralIntakeDIOStbd = 2;
  public static final int algaeIntakeDIO     = 3;

  /* Climber */
  /* _______ */
  public static final int climberWinchMotorID = 20;

  /* Manipulator */
  /* ___________ */
  public static final int coralManipulatorID = 21;
  public static final int algaeManipulatorID = 22;
  
  public static final int canifierID = 23;

  public static final GeneralPin coralManipulatorDIOPort = GeneralPin.SPI_CLK_PWM0P;
  public static final GeneralPin coralManipulatorDIOStbd = GeneralPin.SPI_MOSI_PWM1P;
  public static final GeneralPin algaeManipulatorDIO     = GeneralPin.SPI_MISO_PWM2P;

  /* Vision */
  /* ______ */
  public static final String llPortName = "limelight-port";
  public static final String llStbdName = "limelight-stbd";

}
