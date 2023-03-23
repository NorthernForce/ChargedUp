// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IMU;
import frc.robot.util.FieldDirections;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngleAbs extends PIDCommand {
  /** Creates a new TurnToAngleAbs. */
  public TurnToAngleAbs(Drivetrain drivetrain, IMU imu, FieldDirections direction) {
    super(
        // The controller that the command will use
        new PIDController(2e-2, 0, 0),
        // This should return the measurement
        () -> {
          double currentAngle = MathUtil.inputModulus(imu.getYaw().getDegrees(), -180, 180);
          double error = (currentAngle - direction.getAngle().getDegrees());
          return MathUtil.inputModulus(error, -180, 180);
        },
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.drive(0, output);
        });
      getController().setTolerance(10);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
