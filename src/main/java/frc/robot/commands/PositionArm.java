// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class PositionArm extends ParallelCommandGroup {
  /** Creates a new PositionArm. */
  public PositionArm(Rotation2d armAngle, Rotation2d wristAngle, boolean extendArm) {
    addCommands(
      new SetArmAngle(armAngle),
      new SetWristAngle(wristAngle),
      extendArm ? new ExtendArm() : new RetractArm()
    );
  }
}
