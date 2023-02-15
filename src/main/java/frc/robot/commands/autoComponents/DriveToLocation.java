// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

import java.util.List;
import java.util.function.Supplier;

/**
 * Simple command to drive to a destination using a RamseteCommand
 */
public class DriveToLocation extends RamseteCommand {
  private final Supplier<Boolean> canDrive;
  /**
   * Creates a new DriveToLocation
   * @param newLocation the destination (Pose2d)
   */
  public DriveToLocation(Pose2d newLocation) {
    super(
      TrajectoryGenerator.generateTrajectory(
        navigation.getPose2d(),
        List.of(),
        newLocation,
        new TrajectoryConfig(Constants.MAX_SPEED, Constants.MAX_ACCELERATION)),
      navigation::getPose2d,
      new RamseteController(),
      new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA),
      new DifferentialDriveKinematics(Constants.TRACK_WIDTH),
      drivetrain::getSpeeds,
      new PIDController(Constants.LEFT_DRIVE_PROPORTION, 0, 0),
      new PIDController(Constants.RIGHT_DRIVE_PROPORTION, 0, 0),
      drivetrain::driveVolts,
      drivetrain, navigation
    );
    canDrive = () -> true;
  }
  /**
   * Creates a new DriveToLocation
   * @param newLocation the destination (Pose2d)
   * @param canDrive whether the drivetrain can drive or not
   */
  public DriveToLocation(Pose2d newLocation, Supplier<Boolean> canDrive) {
    super(
      TrajectoryGenerator.generateTrajectory(
        navigation.getPose2d(),
        List.of(),
        newLocation,
        new TrajectoryConfig(Constants.MAX_SPEED, Constants.MAX_ACCELERATION)),
      navigation::getPose2d,
      new RamseteController(),
      new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA),
      new DifferentialDriveKinematics(Constants.TRACK_WIDTH),
      drivetrain::getSpeeds,
      new PIDController(Constants.LEFT_DRIVE_PROPORTION, 0, 0),
      new PIDController(Constants.RIGHT_DRIVE_PROPORTION, 0, 0),
      drivetrain::driveVolts,
      drivetrain, navigation
    );
    this.canDrive = canDrive;
  }
  /**
  * Overrides Ramsete.execute() to only execute if canDrive returns true
  */
  @Override
  public void execute()
  {
    if (canDrive.get()) super.execute();
  }
}