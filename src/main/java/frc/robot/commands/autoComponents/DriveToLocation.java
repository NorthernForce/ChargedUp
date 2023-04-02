// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

import java.util.List;
import java.util.function.Supplier;

/**
 * Simple command to drive to a destination using a RamseteCommand
 */
public class DriveToLocation extends CommandBase {
  private final Supplier<Boolean> canDrive;
  private final RamseteController controller;
  private final double maxVelocity, maxAcceleration;
  private Trajectory trajectory;
  private final Pose2d targetPose;
  private double timeStart;
  /**
   * Creates a new DriveToLocation
   * @param newLocation the destination (Pose2d)
   * @param maxVelocity the maximum velocity (m/s)
   * @param maxAcceleration the maximum acceleration (m^2/s)
   */
  public DriveToLocation(Pose2d newLocation, double maxVelocity, double maxAcceleration) {
    addRequirements(drivetrain, navigation);
    targetPose = newLocation;
    controller = new RamseteController();
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    canDrive = () -> true;
  }
  /**
   * Creates a new DriveToLocation
   * @param newLocation the destination (Pose2d)
   * @param maxVelocity the maximum velocity (m/s)
   * @param maxAcceleration the maximum acceleration (m^2/s)
   * @param canDrive whether the drivetrain can drive or not
   */
  public DriveToLocation(Pose2d newLocation, double maxVelocity, double maxAcceleration, Supplier<Boolean> canDrive) {
    addRequirements(drivetrain, navigation);
    targetPose = newLocation;
    controller = new RamseteController();
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.canDrive = canDrive;
  }
  @Override
  public void initialize() {
    trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(navigation.getPose2d(), targetPose),
      new TrajectoryConfig(maxVelocity, maxAcceleration)
    );
    timeStart = Timer.getFPGATimestamp();
  }
  /**
  * Overrides Ramsete.execute() to only execute if canDrive returns true
  */
  @Override
  public void execute()
  {
    if (canDrive.get())
    {
      Trajectory.State state = trajectory.sample(Timer.getFPGATimestamp() - timeStart);
      ChassisSpeeds speeds = controller.calculate(targetPose, state);
      SmartDashboard.putNumber("Ramsete vx: ", speeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Ramsete vy: ", speeds.vyMetersPerSecond);
      SmartDashboard.putNumber("Ramsete Target Pose x: ", targetPose.getX());
      SmartDashboard.putNumber("Ramsete Target Pose y: ", targetPose.getY());
      SmartDashboard.putNumber("Ramsete State poseMeters X: ", state.poseMeters.getX());
      SmartDashboard.putNumber("Ramsete State poseMeters y: ", state.poseMeters.getY());

      

      drivetrain.driveUsingChassisSpeeds(speeds);
    }
  }
  @Override
  public boolean isFinished() {
    return controller.atReference();
  }
}
