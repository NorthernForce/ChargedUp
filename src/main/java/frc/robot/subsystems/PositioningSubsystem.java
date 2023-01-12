// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.PhotonCamera;

import static frc.robot.RobotContainer.imu;

import java.util.List;

import static frc.robot.RobotContainer.drivetrain;

public class PositioningSubsystem extends SubsystemBase {
  private final DifferentialDrivePoseEstimator estimator;
  private final Pose2d initialPose;
  private final RobotPoseEstimator visionEstimator;
  /** Creates a new PositioningSubsystem. */
  public PositioningSubsystem(List<Pair<PhotonCamera, Transform3d>> cameras) {
    initialPose = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? Constants.bluePoses[DriverStation.getLocation()]
      : Constants.redPoses[DriverStation.getLocation()];
    estimator = new DifferentialDrivePoseEstimator(
      new DifferentialDriveKinematics(Constants.trackWidth), 
      imu.getHeading(), 
      drivetrain.getLeftDistance(), drivetrain.getRightDistance(), initialPose
    );
    visionEstimator = new RobotPoseEstimator(Constants.apriltagLayout, 
      RobotPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, cameras);
  }

  public Pose2d estimatePose()
  {
    return estimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    estimator.update(imu.getHeading(), drivetrain.getLeftDistance(), drivetrain.getRightDistance());
    var data = visionEstimator.update();
    if (data.isPresent())
    {
      estimator.addVisionMeasurement(data.get().getFirst().toPose2d(), data.get().getSecond());
    }
  }
}
