// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Diffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToClimbConfig extends Command {
  /** Creates a new GoToClimbConfig. */
  Diffector s_Diffector;
  public GoToClimbConfig(Diffector s_Diffector) {
    this.s_Diffector = s_Diffector;

    addRequirements(s_Diffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Diffector.goToPosition(0);
    if (s_Diffector.getArmPos() < 10 && s_Diffector.getArmPos() > -10)
    {
      s_Diffector.setElevatorTarget(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
