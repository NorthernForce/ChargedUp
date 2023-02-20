package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(4);
  private final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);
  private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  private PhotonCamera camera = new PhotonCamera(Constants.VISION_CAMERA_NAME);
  private PhotonPipelineResult result = null;
  /** Enum representing the various pipeline IDs */
  public enum VisionPipeline
  {
    YELLOW_CONE,
    PURPLE_CUBE,
    REFLECTIVE_TARGET;
    /** Gets index of pipeline */
    public int getPipelineIndex()
    {
      switch (this)
      {
      case YELLOW_CONE:
        return 0;
      case PURPLE_CUBE:
        return 1;
      case REFLECTIVE_TARGET:
        return 2;
      default:
        return 0;
      }
    }
  };
  /** Creates a new Vision. */
  public Vision() {
  }
  /**
   * Sets the pipeline index
   * @param pipeline VisionPipeline enum
   */
  public void setPipeline(VisionPipeline pipeline)
  {
    camera.setPipelineIndex(pipeline.getPipelineIndex());
  }
  @Override
  public void periodic() {
    result = camera.getLatestResult();
    // This method will be called once per scheduler run
  }
  /**
   * Does the camera see a target
   * @return has target?
   */
  public boolean hasTarget()
  {
    if (result == null) return false;
    return result.hasTargets();
  }
  /**
   * Gets the target yaw
   * @return degree measure
   */
  public Rotation2d getTargetYaw()
  {
    return Rotation2d.fromDegrees(result.getBestTarget().getYaw());
  }
  /**
   * Gets the target pitch
   * @return degree measure
   */
  public Rotation2d getTargetPitch()
  {
    return Rotation2d.fromDegrees(result.getBestTarget().getPitch());
  }
  /**
   * Gets the range of the target
   * @return range in meters
   */
  public double getTargetRange()
  {
    double range = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        TARGET_HEIGHT_METERS,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(result.getBestTarget().getPitch()));
    return range;
  }
}