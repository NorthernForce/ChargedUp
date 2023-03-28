// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.Path;

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
      drivetrain.getFeedforward(),
      drivetrain.getKinematics(),
      drivetrain::getSpeeds,
      new PIDController(Constants.DrivetrainConstants.LEFT_DRIVE_PROPORTION, 0, 0), /** These values don't matter as far as testing shows */
      new PIDController(Constants.DrivetrainConstants.RIGHT_DRIVE_PROPORTION, 0, 0), /** These values don't matter as far as testing shows */
      drivetrain::driveVolts,
      drivetrain, navigation
    );
    canDrive = () -> true;
  }
  public DriveAlongPath(Path path) throws IOException
  {
    super(
      path.load(),
      navigation::getPose2d,
      new RamseteController(),
      drivetrain.getFeedforward(),
      drivetrain.getKinematics(),
      drivetrain::getSpeeds,
      new PIDController(Constants.DrivetrainConstants.LEFT_DRIVE_PROPORTION, 0, 0), /** These values don't matter as far as testing shows */
      new PIDController(Constants.DrivetrainConstants.RIGHT_DRIVE_PROPORTION, 0, 0), /** These values don't matter as far as testing shows */
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
      drivetrain.getFeedforward(),
      drivetrain.getKinematics(),
      drivetrain::getSpeeds,
      new PIDController(Constants.DrivetrainConstants.LEFT_DRIVE_PROPORTION, 0, 0), /** These values don't matter as far as testing shows */
      new PIDController(Constants.DrivetrainConstants.RIGHT_DRIVE_PROPORTION, 0, 0), /** These values don't matter as far as testing shows */
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
