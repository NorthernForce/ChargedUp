// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class SetArmAngle extends CommandBase {
  private final Rotation2d angle;
  /** Creates a new SetArmAngle. */
  public SetArmAngle(Rotation2d angle) {
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armRotate);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armRotate.setAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(armRotate.getAngle().getDegrees()) < Constants.ARM_ANGLE_TOLERANCE;
  }
}
