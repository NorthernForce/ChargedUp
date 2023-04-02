// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class Field {
    public static Pose2d setAlliance(Pose2d bluePos) {
        if ( DriverStation.getAlliance() == Alliance.Blue ) {
            return bluePos;
        }

        double FIELD_LENGTH = 16.4846; //METERS
        double newX = FIELD_LENGTH - bluePos.getX();
        Rotation2d newRotation = Rotation2d.fromDegrees(MathUtil.inputModulus(bluePos.getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees(), -180, 180));
        return new Pose2d(newX, bluePos.getY(), newRotation);
    }
}