// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.led;

public class LEDInit extends CommandBase {
  /** Creates a new LEDInit. */
  public LEDInit() {
    // Use addRequirements() here to declare subsystem dependencies.
    if (led != null) {
      addRequirements(led);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.setPink();
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
