package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private final NetworkTableEntry tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
  private final NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
  private final NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
  private final NetworkTableEntry pipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline");
  private double targetHeight;
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
        return 1;
      case PURPLE_CUBE:
        return 2;
      case REFLECTIVE_TARGET:
        return 0;
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
    this.pipeline.setDouble(pipeline.getPipelineIndex());
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
  }
  /**
   * Does the camera see a target
   * @return has target?
   */
  public boolean hasTarget()
  {
    return tv.getDouble(0.0) == 1.0;
  }
  /**
   * Gets the target yaw (given by Limelight)
   * @return degree measure of yaw
   */
  public Rotation2d getTargetYaw()
  {
    return Rotation2d.fromDegrees(tx.getDouble(0.0));
  }
  /**
   * Gets the target pitch (given by Limelight)
   * @return degree measure of pitch
   */
  public Rotation2d getTargetPitch()
  {
    return Rotation2d.fromDegrees(ty.getDouble(0.0));
  }
  /**
   * Gets the range of the target.
   * Uses trigonometry to calculate the target distance based on the known target height, the height of the vision camera,
   * and the pitches of the camera and the target pitch.
   * @return range in meters
   */
  public double getTargetRange()
  {
    return (targetHeight - Constants.VisionConstants.CAMERA_HEIGHT) / Constants.VisionConstants.CAMERA_PITCH.plus(getTargetPitch()).getTan();
  }
}