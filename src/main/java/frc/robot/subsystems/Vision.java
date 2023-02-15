package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.activeChassis;

public class Vision extends SubsystemBase {
  private final double CAMERA_HEIGHT_METERS = activeChassis.getDoubleConstant("CAMERA_HEIGHT_METERS");
  private final double TARGET_HEIGHT_METERS = activeChassis.getDoubleConstant("TARGET_HEIGHT_METERS");
  private final double CAMERA_PITCH_RADIANS = activeChassis.getDoubleConstant("CAMERA_PITCH_RADIANS");
  private PhotonCamera camera = new PhotonCamera(activeChassis.getStringConstant("VISION_CAMERA_NAME"));
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
  public void setPipeline(VisionPipeline pipeline)
  {
    camera.setPipelineIndex(pipeline.getPipelineIndex());
  }
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
    double range = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        TARGET_HEIGHT_METERS,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(result.getBestTarget().getPitch()));
    return range;
  }
}