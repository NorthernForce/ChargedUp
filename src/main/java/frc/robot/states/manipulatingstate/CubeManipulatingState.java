// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states.manipulatingstate;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

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
    public double getIntakeSetSpeed() {
        return -1.0;
    }

    @Override
    public double getOuttakeSpeed() {
        return 1.0;
    }
}
