// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states.manipulatingstate;

import org.opencv.core.RotatedRect;

import frc.robot.Constants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class ConeManipulatingState implements ManipulatingState {

    @Override
    public Color getColor() {
        return new Color(255, 100, 0);
    }
    
    @Override
    public double getWristDistance() {
        return Units.inchesToMeters(14.5);
    }

    @Override
    public double getLoadingShelfHeight() {
        return 0;
    }

    @Override
    public Rotation2d getLoadingShelfWristAngle() {
        return Rotation2d.fromDegrees(0);
    }

    @Override
    public Pair<Double, Rotation2d> getLoadingStationApproachPosition() {
        return Constants.calculateArmAngleAndDistance(
            Constants.ArmConstants.ORIGIN.getZ(), 
            Constants.ArmConstants.EXTENDED_LENGTH, 
            getWristDistance(), 
            getLoadingShelfWristAngle(), 
            getLoadingShelfHeight(), 
            Constants.ArmConstants.ORIGIN.getX());
    }

    @Override
    public Pair<Double, Rotation2d> getLoadingStationPickUpPosition() {
        return Constants.calculateArmAngleAndDistance(
            Constants.ArmConstants.ORIGIN.getZ(), 
            Constants.ArmConstants.EXTENDED_LENGTH, 
            getWristDistance(), 
            getLoadingShelfWristAngle(), 
            getLoadingShelfHeight(), 
            Constants.ArmConstants.ORIGIN.getX());
    }

    @Override
    public double getIntakeSetSpeed() {
        return 1.0;
    }

    @Override
    public double getOuttakeSpeed() {
        return -1.0;
    }
}
