// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  // Constants such as camera and target height stored. Change per robot and goal!
  private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(4);
  private final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);
  // Angle between horizontal and the camera.
  private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  // How far from the target we want to be
  private final double GOAL_RANGE_METERS = Units.feetToMeters(0.5);

  // Change this to match the name of your camera
  private PhotonCamera camera = new PhotonCamera("apriltagCamera");
  private PhotonPipelineResult result = null;
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    result = camera.getLatestResult();
    // This method will be called once per scheduler run
  }
  public boolean hasTarget()
  {
    if (result == null) return false;
    return result.hasTargets();
  }
  public double getTargetYaw()
  {
    return result.getBestTarget().getYaw();

  }
  public double getTargetPitch()
  {
    return result.getBestTarget().getPitch();
  }
  public double getTargetRange()
  {
    double range =
    PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(result.getBestTarget().getPitch()));
    return range;

  }
}