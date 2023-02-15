// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.activeChassis;

import static frc.robot.RobotContainer.drivetrain;

public class SlowMode extends CommandBase {
  /** Creates a new SlowMode. */
  public SlowMode() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setSpeedProportions(activeChassis.getDoubleConstant("SLOW_SPEED_FORWARD"), activeChassis.getDoubleConstant("SLOW_SPEED_ROTATION"));
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setSpeedProportions(activeChassis.getDoubleConstant("FAST_SPEED_FORWARD"), activeChassis.getDoubleConstant("FAST_SPEED_ROTATION"));
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
