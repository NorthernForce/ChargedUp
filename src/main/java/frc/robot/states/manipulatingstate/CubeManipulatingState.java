// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states.manipulatingstate;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

/** Add your docs here. */
public class CubeManipulatingState implements ManipulatingState {

    @Override
    public Color getColor() {
        return Color.kPurple;
    }

    @Override
    public double getWristDistance() {
        return Units.inchesToMeters(9);
    }

    @Override
    public double getLoadingShelfHeight() {
        return 0;
    }
    
    @Override
    public Rotation2d getLoadingShelfWristAngle() {
        return Rotation2d.fromDegrees(0);
    }

    // Get the distance and arm angle we need to be in to pick up
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
        return -1.0;
    }

    @Override
    public double getOuttakeSpeed() {
        return 1.0;
    }
}
