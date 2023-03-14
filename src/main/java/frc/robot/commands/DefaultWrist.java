// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

import static frc.robot.RobotContainer.*;

import java.util.function.DoubleSupplier;

public class DefaultWrist extends CommandBase {
  /** Creates a new DefaultWrist. */
  DoubleSupplier[] manipulatorSuppliers;
  public DefaultWrist() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
    this.manipulatorSuppliers = OI.getManipulatorSuppliers();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setPercent(manipulatorSuppliers[2].getAsDouble());
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
