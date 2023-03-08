// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmAngle extends CommandBase {
  private final Rotation2d angle;
  /** Creates a new SetArmAngle. */
  public SetArmAngle(Rotation2d angle) {
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armRotate);
    // Configure additional PID options by calling `getController` here.
  }
  @Override
  public void initialize()
  {
  }
  @Override
  public void execute()
  {
    armRotate.setArmPosition(angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
