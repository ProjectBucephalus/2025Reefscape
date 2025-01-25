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
    double maxAbsPos = Constants.ArmControl.maxAbsPos;
    double maxRotations = Constants.ArmControl.maxRotations;
    double turnBackThreshold = Constants.ArmControl.turnBackThreshold;
    double stowThreshold = Constants.ArmControl.stowThreshold;
    double offset;
    double altOffset;

    /** 
     * Sets the Diffector arm to unwind to starting position 
     * @return Safe to stow
     */
    public boolean unwind()
    {
        targetPos = Constants.ArmControl.returnPos;
        return (Math.abs(currentPos) < stowThreshold);
    }

    /**
     * Sets the Diffector arm to rotate the shortest path to the target angle, with protection against over-rotation
     * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
     */
    public void goShortest(double targetAngle)
    {
        targetAngle %= 360;
        offset = MathUtil.inputModulus(targetAngle - (currentPos % 360), -180, 180);


        if (currentPos + offset > maxAbsPos)
            {targetPos = (currentPos + offset - 360);}

        else if (currentPos + offset < -maxAbsPos)
            {targetPos = (currentPos + offset + 360);}

        else
            {targetPos = (currentPos + offset);}
    }

    /**
     * Sets the Diffector arm to rotate Clockwise (viewed from bow) to the target angle, with protection against over-rotation
     * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
     */
    public void goClockwise(double targetAngle)
    {
        targetAngle %= 360;
        offset = MathUtil.inputModulus(targetAngle - (currentPos % 360), -360, 0);


        if (currentPos + offset > maxAbsPos)
            {targetPos = (currentPos + offset - 360);}

        else if (currentPos + offset < -maxAbsPos)
            {targetPos = (currentPos + offset + 360);}

        else
            {targetPos = (currentPos + offset);}
    }

    /**
     * Sets the Diffector arm to rotate Anticlockwise (viewed from bow) to the target angle, with protection against over-rotation
     * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
     */
    public void goAnticlockwise(double targetAngle)
    {
        targetAngle %= 360;
        offset = MathUtil.inputModulus(targetAngle - (currentPos % 360), 0, 360);


        if (currentPos + offset > maxAbsPos)
            {targetPos = (currentPos + offset - 360);}

        else if (currentPos + offset < -maxAbsPos)
            {targetPos = (currentPos + offset + 360);}

        else
            {targetPos = (currentPos + offset);}
    }

    /**
     * Sets the Diffector arm to rotate the safest path to the target angle, with protection against over-rotation. 
     * Below a threshold will go shortest path, otherwise will minimise total rotations
     * @param targetAngle Target angle of the arm, degrees anticlockwise, 0 = coral at top
     */
    public void goToPosition(double targetAngle)
    {
        targetAngle %= 360;
        offset = MathUtil.inputModulus(targetAngle - (currentPos % 360), -180, 180);

        if (Math.abs(offset) >= 135)
        {
            altOffset = offset - Math.copySign(360, offset);

            if (Math.abs(currentPos + offset) > Math.abs(currentPos + altOffset))
                {targetPos = (currentPos + altOffset);}
            
            else 
                {targetPos = (currentPos + offset);}
        }
        else if (currentPos + offset > maxAbsPos)
            {targetPos = (currentPos + offset - 360);}

        else if (currentPos + offset < -maxAbsPos)
            {targetPos = (currentPos + offset + 360);}

        else
            {targetPos = (currentPos + offset);}
    }

}