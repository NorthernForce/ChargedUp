// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states.manipulatingstate;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class EmptyManipulatingState implements ManipulatingState {

    @Override
    public Color getColor() {
        return new Color(255, 0, 17);
    }

    @Override
    public double getWristDistance() {
        return 0;
    }

    @Override
    public double getLoadingShelfHeight() {
        return 0;
    }

    @Override
    public double getSlideHeight() {
        return 0;
    }
    
    @Override
    public Rotation2d getLoadingShelfWristAngle() {
        return null;
    }

    @Override
    public Rotation2d getSlideWristAngle() {
        return null;
    };

    @Override
    public Pair<Double, Rotation2d> getLoadingStationApproachPosition() {
        return null;
    }

    @Override
    public Pair<Double, Rotation2d> getLoadingStationPickUpPosition() {
        return null;
    }

    @Override
    public Pair<Double, Rotation2d> getSlideApproachPosition()
    {
        return null;
    }

    @Override
    public Pair<Double, Rotation2d> getSlidePickUpPosition() {
        return null;
    }

    @Override
    public double getIntakeSetSpeed() {
        return 0;
    }

    @Override
    public double getOuttakeSpeed() {
        return 0;
    }
}
