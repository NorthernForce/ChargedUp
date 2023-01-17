// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;
/**
 * A simple command to stop the robot.
 */
public class Stop extends CommandBase {
  private final double tolerance;
  /**
   * Creates a new Stop.
   * @param tolerance the amount of tolerance for speed in m/s
   */
  public Stop(double tolerance) {
    addRequirements(drivetrain);
    this.tolerance = tolerance;
  }
  /**
   * Implements CommandBase.initialize().
   */
  @Override
  public void initialize() {}
  /**
   * Implements CommandBase.execute(). Sets speeds to 0.
   */
  @Override
  public void execute() {
    drivetrain.driveUsingSpeeds(0, 0);
  }
  /**
   * Implements CommandBase.end(boolean).
   */
  @Override
  public void end(boolean interrupted) {}
  /**
   * Implements CommandBase.isFinished(). Returns whether speeds are within tolerance.
   */
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getSpeeds().leftMetersPerSecond) <= tolerance
      && Math.abs(drivetrain.getSpeeds().rightMetersPerSecond) <= tolerance;
  }
}
