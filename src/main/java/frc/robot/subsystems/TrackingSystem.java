// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrackingSystem extends SubsystemBase {
  public enum CameraFilter
  {
    YELLOW_BALL,
    APRILTAG;
    public int getPipelineIndex()
    {
      switch (this)
      {
      case YELLOW_BALL:
        return 0;
      case APRILTAG:
        return 1;
      default:
        return -1;
      }
    }
  }
  private final PhotonCamera camera;
  private PhotonPipelineResult lastResult;
  /** Creates a new TrackingSystem. */
  public TrackingSystem(String cameraName, CameraFilter filter) {
    camera = new PhotonCamera(cameraName);
    camera.setPipelineIndex(filter.getPipelineIndex());
  }
  void setFilter(CameraFilter filter)
  {
    camera.setPipelineIndex(filter.getPipelineIndex());
  }
  @Override
  public void periodic() {
    lastResult = camera.getLatestResult();
  }
  public boolean hasTargets()
  {
    return lastResult != null && lastResult.hasTargets();
  }
  public PhotonTrackedTarget getBestTarget()
  {
    if (lastResult == null) return null;
    return lastResult.getBestTarget();
  }
  public double getTargetYawDegrees()
  {
    assert hasTargets();
    return getBestTarget().getYaw();
  }
  /* REQUIRES 3D CALIBRATION! */
  public Transform3d getTransformToTarget()
  {
    assert hasTargets();
    return lastResult.getBestTarget().getBestCameraToTarget();
  }
  /* Pythagorean theorem: REQUIRES 3D CALIBRATION! */
  public double estimateRangeMeters()
  {
    Transform3d targetTransform = getTransformToTarget();
    double x = targetTransform.getX(), y = targetTransform.getY();
    return Math.sqrt(x * x + y * y);
  }
}
