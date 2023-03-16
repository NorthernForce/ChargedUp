// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states.manipulatingstate;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public interface ManipulatingState {
    Color getColor();
    double getWristDistance();
}
