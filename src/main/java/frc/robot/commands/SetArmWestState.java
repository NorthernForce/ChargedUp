// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class SetArmWestState extends ParallelCommandGroup {
  /** Creates a new SetArmEastState. */
  public SetArmWestState() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new SetArmAngle(Constants.ArmConstants.WEST_ANGLE),
      new SetWristAngle(Constants.WristConstants.WEST_ANGLE)
    );
  }
}
