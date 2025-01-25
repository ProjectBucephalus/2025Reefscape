// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Diffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Diffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToStartConfig extends Command {
  /** Creates a new GoToStartConfig. */
  Diffector s_Diffector;
  private int step;
  public GoToStartConfig(Diffector s_Diffector) {
    this.s_Diffector = s_Diffector;

    addRequirements(s_Diffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {step = 0;}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    switch (step) {
      case 0:
        if (s_Diffector.getElevatorPos() >= 0.5)
        {
          step += 1;
        } else 
        {
          s_Diffector.setElevatorTarget(0.6);
        }
        break;

      case 1:
        s_Diffector.unwind();
        if (s_Diffector.unwind())
        {
          step += 1;
        }
        break;

      case 2:
        s_Diffector.setElevatorTarget(0);
        break;
    
      default:
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return step == 2;
  }
}
