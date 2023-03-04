// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public enum FieldDirections {
    AWAY,
    LEFT,
    RIGHT,
    RETURN;

    public Rotation2d getAngle() {
        Alliance alliance = DriverStation.getAlliance();
        int alliance_offset = (alliance == Alliance.Red) ? 0 : 180;
        int angle;
        switch (this) {  
            case AWAY:
                angle = 0;
                break;
            case LEFT:
                angle = 90;
                break;
            case RIGHT:
                angle = 270;
                break;
            case RETURN:
                angle = 180;
                break;
            default:
                angle = 0;
                break;
        }
        double degrees = MathUtil.inputModulus((angle + alliance_offset), -180, 180);
        return new Rotation2d(Math.toRadians(degrees));
    }
}
