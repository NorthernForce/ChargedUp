// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmAngle extends ProfiledPIDCommand {
  /** Creates a new SetArmAngle. */
  public SetArmAngle(Rotation2d angle) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.ARM_PROPORTION,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.ARM_MAX_RADIANS_PER_SECOND, Constants.ARM_MAX_RADIANS_PER_SECOND_SQUARED)),
        // This should return the measurement
        () -> armRotate.getAngle().getDegrees(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(angle.getDegrees(), 0),
        // This uses the output
        (output, setpoint) -> {
          armRotate.setArmVoltage(output + armRotate.getFeedforward().calculate(Math.toRadians(setpoint.position), setpoint.velocity));
          // Use the output (and setpoint, if desired) here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armRotate);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
