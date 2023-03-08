// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import static frc.robot.RobotContainer.*;

public class TurnToTarget extends PIDCommand {
  /** Creates a new TurnToTarget. */

  public TurnToTarget() {
    super(
      new PIDController(
        2e-2,
        2e-3,
        0
      ),
      () -> 0,
      () -> vision.getTargetYaw().getDegrees(),
      (output) -> drivetrain.drive(0, output),
      drivetrain, vision);
  }
  @Override
  public boolean isFinished() {
    return Math.abs(vision.getTargetYaw().getDegrees()) < 3;
  }
}
