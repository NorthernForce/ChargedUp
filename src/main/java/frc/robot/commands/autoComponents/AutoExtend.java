// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ExtendArm;

import static frc.robot.RobotContainer.*;

public class AutoExtend extends SequentialCommandGroup {
  /** Creates a new AutoExtend. */
  public AutoExtend() {
    addCommands(
      Commands.deadline(Commands.waitUntil(() -> armTelescope.isExtended()), Commands.repeatingSequence(new ExtendArm())),
      Commands.waitSeconds(1.5)
    );
  }
}
