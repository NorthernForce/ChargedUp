package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private final double CAMERA_HEIGHT_METERS = Constants.VISION_CAMERA_HEIGHT;
  private double targetHeight = 0.0;
  private final double CAMERA_PITCH_RADIANS = Constants.VISION_CAMERA_PITCH.getRadians();
  private final PhotonCamera camera = new PhotonCamera(Constants.VISION_CAMERA_NAME);
  private PhotonPipelineResult result = null;
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
   * @param pipeline VisionPipeline instance
   */
  public void setPipeline(VisionPipeline pipeline)
  {
    camera.setPipelineIndex(pipeline.getPipelineIndex());
  }
  /**
   * Sets the target height
   * @param targetHeight height in meters
   */
  public void setTargetHeight(double targetHeight)
  {
    this.targetHeight = targetHeight;
  }
  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    result = camera.getLatestResult();
  }
  /**
   * Checks to see whether there is a target.
   * @return whether the camera can see a target
   */
  public boolean hasTarget()
  {
    if (result == null) return false;
    return result.hasTargets();
  }
  /**
   * Gets the target yaw (given by Photonvision)
   * @return degree measure of yaw
   */
  public double getTargetYaw()
  {
    return result.getBestTarget().getYaw();
  }
  /**
   * Gets the target pitch (given by Photonvision)
   * @return degree measure of pitch
   */
  public double getTargetPitch()
  {
    return result.getBestTarget().getPitch();
  }
  /**
   * Gets the target range
   * @return distance in meters
   */
  public double getTargetRange()
  {
    double range = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        targetHeight,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(result.getBestTarget().getPitch()));
    return range;
  }
}