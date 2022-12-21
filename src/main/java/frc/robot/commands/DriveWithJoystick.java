// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

import static frc.robot.RobotContainer.drivetrain;

import java.util.function.DoubleSupplier;

public class DriveWithJoystick extends CommandBase {

  private DoubleSupplier[] Speeds = OI.getDriveSuppliers();
  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(Speeds[0].getAsDouble(), Speeds[1].getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
