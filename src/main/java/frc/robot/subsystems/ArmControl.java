// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO: -Shortest Path
//      -Choose Path
//      -Go to (Shortest Path / Toward Zero)
//      -Unwind Command (Put In Commands Folder)

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ArmControl extends SubsystemBase 
{
    double rotationCount;
    double currentPos;
    double targetPos;
    double calcTargetPos;
    double maxAbsPos = Constants.ArmControl.maxAbsPos;
    double maxRotations = Constants.ArmControl.maxRotations;
    double offset;
    double altOffset;
    double turnBackThreshold = Constants.ArmControl.turnBackThreshold;

    private void unwind()
        {targetPos = Constants.ArmControl.returnPos;}

    private double goShortest(double targetPos)
    {
        offset = MathUtil.inputModulus(targetPos - (currentPos % 360), -180, 180);


        if (currentPos + offset > maxAbsPos)
            {return (currentPos + offset - 360);}

        else if (currentPos + offset < -maxAbsPos)
            {return (currentPos + offset + 360);}

        else
            {return (currentPos + offset);}
    }

    private double goClockwise(double targetPos)
    {
        offset = MathUtil.inputModulus(targetPos - (currentPos % 360), -360, 0);


        if (currentPos + offset > maxAbsPos)
            {return (currentPos + offset - 360);}

        else if (currentPos + offset < -maxAbsPos)
            {return (currentPos + offset + 360);}

        else
            {return (currentPos + offset);}
    }

    private double goAnticlockwise(double targetPos)
    {
        offset = MathUtil.inputModulus(targetPos - (currentPos % 360), 0, 360);


        if (currentPos + offset > maxAbsPos)
            {return (currentPos + offset - 360);}

        else if (currentPos + offset < -maxAbsPos)
            {return (currentPos + offset + 360);}

        else
            {return (currentPos + offset);}
    }

    private double goToPosition(double targetPos)
    {
        offset = MathUtil.inputModulus(targetPos - (currentPos % 360), -180, 180);

        if (Math.abs(offset) >= 135)
        {
            altOffset = offset - Math.copySign(360, offset);

            if (Math.abs(currentPos + offset) > Math.abs(currentPos+altOffset))
                {return (currentPos + altOffset);}
            
            else 
                {return (currentPos + offset);}
        }
        
        if (currentPos + offset > maxAbsPos)
            {return (currentPos + offset - 360);}

        else if (currentPos + offset < -maxAbsPos)
            {return (currentPos + offset + 360);}

        else
            {return (currentPos + offset);}
    }
}