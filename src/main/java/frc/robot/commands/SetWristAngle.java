// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class SetWristAngle extends CommandBase {
  private final Rotation2d targetAngle;
  /** Creates a new SetWristAngle. */
  public SetWristAngle(Rotation2d targetAngle) {
    this.targetAngle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setRotation(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(armRotate.getAngle().minus(targetAngle).getDegrees()) < Constants.ArmConstants.ANGLE_TOLERANCE;
  }
}
