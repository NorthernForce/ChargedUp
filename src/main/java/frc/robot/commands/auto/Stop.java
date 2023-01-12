// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class Stop extends CommandBase {
  private final double tolerance;
  /** Creates a new Stop. */
  public Stop(double tolerance) {
    addRequirements(drivetrain);
    this.tolerance = tolerance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveUsingSpeeds(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getSpeeds().leftMetersPerSecond) <= tolerance
      && Math.abs(drivetrain.getSpeeds().rightMetersPerSecond) <= tolerance;
  }
}
