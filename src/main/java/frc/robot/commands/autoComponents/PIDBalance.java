// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;
/**
 * A Command to automatically balance on the Charge Station.
 */
public class PIDBalance extends CommandBase {
  private PIDController controller;
  /** Creates a new PIDBalance. */
  public PIDBalance() {
    System.err.println("Balance do be constructing tho");
    addRequirements(drivetrain, navigation);
  }
  /**
   * Implements CommandBase.initialize(). Initializes PIDController.
   */
  @Override
  public void initialize() {
    System.err.println("Im gonna blance :)");
    controller = new PIDController(2e-2, 0, 0);
    controller.setSetpoint(0);
  }
  /**
   * Implements CommandBase.execute(). Drives forward or back at the required speed to reach
   * a pitch of 0.
   */
  @Override
  public void execute() {
    System.err.println("Im doing it!!!");
    double forwardSpeed = -controller.calculate(Math.toDegrees(imu.getPitch()));

    SmartDashboard.putNumber("Pitch", Math.toDegrees(imu.getPitch()));
    SmartDashboard.putNumber("PID Output", forwardSpeed);

    drivetrain.drive(forwardSpeed, 0);
  }
  /**
   * Implements CommandBase.end(boolean)
   */
  @Override
  public void end(boolean interrupted) {
    System.err.println("Im donneee");
  }
  /**
   * Implements CommandBase.isFinished(). Always returns false.
   */
  @Override
  public boolean isFinished() {
    // return controller.atSetpoint();
    return false;
  }
}
