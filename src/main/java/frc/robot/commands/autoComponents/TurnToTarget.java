// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class TurnToTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private final int cameraIdx;

  public TurnToTarget(int cameraIdx) {
    this.cameraIdx = cameraIdx;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.hasTarget(cameraIdx)){
      double requiredTurn = (vision.getTargetYaw(cameraIdx).getDegrees() > 0) ? -0.4 : 0.4;
      drivetrain.drive(0, requiredTurn);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(vision.getTargetYaw(cameraIdx).getDegrees()) < 3;
  }
}
