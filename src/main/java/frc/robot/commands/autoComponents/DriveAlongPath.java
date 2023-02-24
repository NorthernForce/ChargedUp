// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * Simple command to drive along a path of translations using a RamseteCommand
 */
public class DriveAlongPath extends RamseteCommand {
  private final Supplier<Boolean> canDrive;
  /**
   * Creates a new DriveAlongPath
   * @param pathName name of the pathweaver json
   * @throws IOException
   */
  public DriveAlongPath(String pathName) throws IOException {
    super(
      TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName + ".wpilib.json")
      ),
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
   * Creates a new DriveAlongPath
   * @param pathName name of the pathweaver json
   * @param canDrive a supplier to check whether the drivetrain can drive or not
   */
  public DriveAlongPath(String pathName, Supplier<Boolean> canDrive) throws IOException {
    super(
      TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName + ".wpilib.json")
      ),
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
