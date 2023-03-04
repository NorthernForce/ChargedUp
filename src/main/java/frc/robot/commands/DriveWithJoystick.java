// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

import static frc.robot.RobotContainer.drivetrain;

import java.util.function.DoubleSupplier;

/**
 * Drives based on controller input.
 */
public class DriveWithJoystick extends CommandBase {
  private DoubleSupplier[] driverSpeeds = OI.getDriveSuppliers();
  private DoubleSupplier[] manipulatorSpeeds = OI.getDriveSuppliers();
  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick() {
    addRequirements(drivetrain);
  }
  @Override
  public void initialize() {}
  /**
   * Drives based on controller input.
   */
  @Override
  public void execute() {
    drivetrain.drive(driverSpeeds[0].getAsDouble(), driverSpeeds[1].getAsDouble());
  }
  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {
    return false;
  }
}
