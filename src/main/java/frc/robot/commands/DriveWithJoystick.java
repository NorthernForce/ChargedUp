// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

import static frc.robot.RobotContainer.drivetrain;

import java.util.function.DoubleSupplier;

/**
 * Drives based on controller input.
 */
public class DriveWithJoystick extends CommandBase {
  private DoubleSupplier[] Speeds = OI.getDriveSuppliers();
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
    if (manipulatorSpeeds[1].getAsDouble() == 0)
    {
      drivetrain.drive(Speeds[0].getAsDouble(), Speeds[1].getAsDouble());
    }
    else
    {
      drivetrain.drive(Speeds[0].getAsDouble(), manipulatorSpeeds[1].getAsDouble() * 0.4);
    }
  }
  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {
    return false;
  }
}
