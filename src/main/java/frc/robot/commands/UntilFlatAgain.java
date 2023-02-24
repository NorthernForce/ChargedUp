// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.imu;

public class UntilFlatAgain extends CommandBase {
  private boolean level;
  private boolean hasTipped = false;
  /** Creates a new ScheduleUntilFlatAgain. */
  public UntilFlatAgain() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    level = (Math.abs(imu.getPitch()) < 6);
    if (!level) {
      hasTipped = true;
    }
    SmartDashboard.putBoolean("Level: ", level);
    SmartDashboard.putBoolean("Has Tipped: ", hasTipped);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (level && hasTipped);
  }
}
