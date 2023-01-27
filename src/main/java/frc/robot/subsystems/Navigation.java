// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import static frc.robot.RobotContainer.*;

/** Simple subsystem to keep track of the current location of the robot. */
public class Navigation extends SubsystemBase {
  private final AHRS ahrs;
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator visionEstimator;
  /** Creates a new Navigation. */
  public Navigation(Pose2d initialPose, String cameraName,
    Transform3d robotToCamera) {
    ahrs = new AHRS();
    ahrs.reset();
    drivetrain.resetEncoderRotations();
    poseEstimator = new DifferentialDrivePoseEstimator(
      new DifferentialDriveKinematics(Constants.trackWidth),
      ahrs.getRotation2d(),
      drivetrain.getLeftDistance(),
      drivetrain.getRightDistance(),
      initialPose
    );
    camera = new PhotonCamera(cameraName);
    visionEstimator = new PhotonPoseEstimator(
      Constants.apriltagLayout,
      PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
      camera,
      robotToCamera
    );
  }
  /**
   * Gets the current location in Meters
   * @return a Pose2d object
   */
  public Pose2d getPose2d()
  {
    return poseEstimator.getEstimatedPosition();
  }
  /**
   * Gets the rotation3d reported by the NavX
   * @return Rotation3d object in Radians
   */
  public Rotation3d getRotation()
  {
    return new Rotation3d(
      Math.toRadians(ahrs.getRoll()),
      Math.toRadians(ahrs.getPitch()),
      Math.toRadians(ahrs.getYaw())
    );
  }
  /** Calibrates the NavX */
  public void calibrate()
  {
    ahrs.calibrate();
  }
  /**
   * Gets whether the NavX is Calibrating
   * @return whether the NavX is Calibrating
   */
  public boolean isCalibrating()
  {
    return ahrs.isCalibrating();
  }
  /** Runs once every 20ms. */
  @Override
  public void periodic() {
    poseEstimator.update(
      ahrs.getRotation2d(),
      drivetrain.getLeftDistance(),
      drivetrain.getRightDistance()
    );
    var results = visionEstimator.update();
    if (results.isPresent())
    {
      poseEstimator.addVisionMeasurement(
        results.get().estimatedPose.toPose2d(),
        results.get().timestampSeconds
      );
    }
  }
}