// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Diffector.GoToAlgaeIntakePos;
import frc.robot.commands.Auto.PathfindToReef.DpadOptions;
import frc.robot.commands.Diffector.GoToCoralScorePos;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.commands.Manipulator.*;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Diffector;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.CoralManipulator.CoralManipulatorStatus;
import frc.robot.util.FieldUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreSequence extends SequentialCommandGroup 
{
  private boolean coral;
  private boolean algae;
  private boolean algaeLevel2;
  private int nearestReefFace;
  private DpadOptions postSide;
  private int coralLevel;

  private Translation2d robotPos;

  public AutoScoreSequence(Diffector s_Diffector, AlgaeManipulator s_AlgaeManipulator, CoralManipulator s_CoralManipulator, CommandSwerveDrivetrain s_Swerve, Supplier<Translation2d> posSup) 
  {
    coral = s_CoralManipulator.getStatus() == CoralManipulatorStatus.DEFAULT;
    algae = s_AlgaeManipulator.getStatus() == AlgaeManipulatorStatus.HOLDING;
    robotPos = posSup.get();
    nearestReefFace = FieldUtils.getNearestReefFace(robotPos);

    if (RobotContainer.driver.povLeft().getAsBoolean()) 
      {postSide = DpadOptions.LEFT;}
    else if (RobotContainer.driver.povRight().getAsBoolean())
      {postSide = DpadOptions.RIGHT;}
    else 
    {
      switch (nearestReefFace) 
      {
        case 2:
        case 3:
        case 4:
          postSide = DpadOptions.RIGHT;
          break;
      
        case 1:
        case 6:
        case 5:
          postSide = DpadOptions.LEFT;
          break;
      }
    }

    if (RobotContainer.copilot.a().getAsBoolean()) 
      {coralLevel = 1;}
    else if (RobotContainer.copilot.b().getAsBoolean())
      {coralLevel = 2;}
    else if (RobotContainer.copilot.x().getAsBoolean())
      {coralLevel = 3;}
    else if (RobotContainer.copilot.y().getAsBoolean())
      {coralLevel = 4;}
    else if (algaeLevel2) 
      {coralLevel = 3;}
    else if (!algaeLevel2) 
      {coralLevel = 2;}

    switch (nearestReefFace) 
    {
      case 1:
      case 3:
      case 5:
        algaeLevel2 = true;
        break;
    
      case 2:
      case 4:
      case 6:
        algaeLevel2 = false;
        break;
    }
    
    if (coral && !algae) 
    {
      addCommands
      (
        new PathfindToReef(DpadOptions.CENTRE, posSup, s_Swerve)
        .alongWith(new GoToAlgaeIntakePos(algaeLevel2, s_Diffector)),
        
        new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.INTAKE),
        
        new PathfindToReef(postSide, posSup, s_Swerve)
        .alongWith(new GoToCoralScorePos(coralLevel, s_Diffector)),

        new SetCoralStatus(s_CoralManipulator, CoralManipulatorStatus.DELIVERY_LEFT),

        new MoveTo(s_Diffector, Constants.DiffectorConstants.algaeStowPosition)
      );
    }
  }
}
