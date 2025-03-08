// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Diffector.MoveTo;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.AlgaeManipulator.AlgaeManipulatorStatus;
import frc.robot.subsystems.Diffector;
import frc.robot.util.FieldUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectAlgaeSmart extends SequentialCommandGroup
{
  private int nearestReefFace;
  private Translation2d robotPos;
  private Translation2d target;

  public EjectAlgaeSmart(AlgaeManipulator s_AlgaeManipulator, Diffector s_Diffector, Supplier<Translation2d> posSup) 
  {
    robotPos = posSup.get();
    nearestReefFace = FieldUtils.getNearestReefFace(robotPos);

    switch (nearestReefFace) 
    {
      case 1:
      case 3:
        target = Constants.DiffectorConstants.algae3StbdPosition;
        break;

      case 2:
      case 4:
        target = Constants.DiffectorConstants.algae2StbdPosition;
        break;

      case 5:
        target = Constants.DiffectorConstants.algae3PortPosition;
        break;

      case 6:
        target = Constants.DiffectorConstants.algae2PortPosition;
        break;
    
      default:
        target = s_Diffector.getRelativeTarget();
        break;
    }

    addCommands(new MoveTo(s_Diffector, target), new WaitUntilCommand(() -> RobotContainer.s_Diffector.atPosition(target)), new SetAlgaeStatus(s_AlgaeManipulator, AlgaeManipulatorStatus.EJECT));
  }
}
