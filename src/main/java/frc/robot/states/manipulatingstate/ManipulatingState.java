// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states.manipulatingstate;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public interface ManipulatingState {

    /**
     * Used to get the color associated with the manipulating state.
     * @return The corresponding color
     */
    public Color getColor();

    /**
     * Returns how far the manipulating type is away from the wrist fulcrum.
     * @return distance in meters
     */
    public double getWristDistance();

    /**
     * Returns the height that the arm needs to go to for the loading stations
     * @return height in meters
     */
    public double getLoadingShelfHeight();

    /**
     * Returns the height that the arm needs to go to for the slide station
     * @return height in meters
     */
    public double getSlideHeight();

    /**
     * Returns the angle to set the wrist at when loading pieces from the shelf.
     * @return Angle stored in Rotation2d
     */
    public Rotation2d getLoadingShelfWristAngle();

    /** Returns the angle to set the wrist at when loading pieces from the shelf.
     * @return Angle stored in Rotation2d
     */
    public Rotation2d getSlideWristAngle();

    // Get the distance and arm angle we need to be in to pick up
    public Pair<Double, Rotation2d> getLoadingStationApproachPosition();

    public Pair<Double, Rotation2d> getLoadingStationPickUpPosition();

    public Pair<Double, Rotation2d> getSlideApproachPosition();

    public Pair<Double, Rotation2d> getSlidePickUpPosition();

    /**
     * Returns the speed that the intake motors should be run at
     * @return [-1.0, 1.0] for the motor
     */
    public double getIntakeSetSpeed();

    /**
     * Returns the speed that the outtake motors should be run at
     * @return [-1.0, 1.0] for the motor
     */
    public double getOuttakeSpeed();

}
