// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states.manipulatingstate;

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
