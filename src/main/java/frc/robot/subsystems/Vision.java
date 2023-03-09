package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CameraWrapper;
import frc.robot.util.LimelightWrapper;

public class Vision extends SubsystemBase {
  private final List<CameraWrapper> cameras;
  /** Creates a new Vision. */
  public Vision() {
    cameras = List.of(
      new LimelightWrapper(Constants.VISION_TRANSFORM3D)
    );
  }
  /**
   * Sets the pipeline index
   * @param pipeline VisionPipeline enum
   */
  public void setPipeline(int cameraIdx, int pipeline)
  {
    cameras.get(cameraIdx).setPipelineIndex(pipeline);
  }
  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
  }
  /**
   * Does the camera see a target
   * @return has target?
   */
  public boolean hasTarget(int cameraIdx)
  {
    return cameras.get(cameraIdx).hasTarget();
  }
  /**
   * Gets the target yaw (given by Limelight)
   * @return degree measure of yaw
   */
  public Rotation2d getTargetYaw(int cameraIdx)
  {
    return cameras.get(cameraIdx).getTargetYaw();
  }
  /**
   * Gets the target pitch (given by Limelight)
   * @return degree measure of pitch
   */
  public Rotation2d getTargetPitch(int cameraIdx)
  {
    return cameras.get(cameraIdx).getTargetPitch();
  }
  /**
   * Gets the range of the target.
   * Uses trigonometry to calculate the target distance based on the known target height, the height of the vision camera,
   * and the pitches of the camera and the target pitch.
   * @return range in meters
   */
  public double getTargetRange(int cameraIdx, double estimatedTargetHeight)
  {
    return cameras.get(cameraIdx).getTargetDistance(estimatedTargetHeight);
  }
}