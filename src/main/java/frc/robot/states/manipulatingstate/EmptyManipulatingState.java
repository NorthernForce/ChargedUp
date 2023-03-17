// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states.manipulatingstate;

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
    public double getIntakeSetSpeed() {
        return 0;
    }

    @Override
    public double getOuttakeSpeed() {
        return 0;
    }
}
