// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class DriveDistanceFromCoordinates extends ProfiledPIDCommand {
  /** Creates a new DriveDistanceFromCoordinates.
   * Assumes aligned with coordinates.
   */
  public DriveDistanceFromCoordinates(double distance, Translation2d coordinates) {
    super(
      new ProfiledPIDController(
        2e-1, 2e-2, 0, new Constraints(Constants.DrivetrainConstants.MAX_SPEED, Constants.DrivetrainConstants.MAX_ACCELERATION)
      ),
      () -> navigation.getPose2d().getTranslation().getDistance(coordinates),
      distance,
      (output, state) -> drivetrain.driveVolts(
        -(output + drivetrain.getFeedforward().calculate(state.velocity)),
        -(output + drivetrain.getFeedforward().calculate(state.velocity))
      ),
      navigation, drivetrain
    );
  }
}
