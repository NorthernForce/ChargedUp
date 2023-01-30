// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;
/**
 * A Command to automatically balance on the Charge Station.
 */
public class PIDBalance extends CommandBase {
  private PIDController controller;
  /** Creates a new PIDBalance. */
  public PIDBalance() {
    addRequirements(drivetrain, navigation);
  }
  /**
   * Implements CommandBase.initialize(). Initializes PIDController.
   */
  @Override
  public void initialize() {
    controller = new PIDController(2e-2, 0, 0);
    controller.setSetpoint(0);
  }
  /**
   * Implements CommandBase.execute(). Drives forward or back at the required speed to reach
   * a pitch of 0.
   */
  @Override
  public void execute() {
    double forwardSpeed = -controller.calculate(Math.toDegrees(imu.getPitch()));
    drivetrain.drive(forwardSpeed, 0);
  }
  /**
   * Implements CommandBase.end(boolean)
   */
  @Override
  public void end(boolean interrupted) {}
  /**
   * Implements CommandBase.isFinished(). Always returns false.
   */
  @Override
  public boolean isFinished() {
    // return controller.atSetpoint();
    return false;
  }
}
