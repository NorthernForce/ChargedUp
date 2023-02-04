package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DynamicTransform3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import static frc.robot.RobotContainer.*;

/** Simple subsystem to keep track of the current location of the robot. */
public class Navigation extends SubsystemBase {
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final PhotonPoseEstimator visionEstimator;
  private final PhotonCamera camera = new PhotonCamera("apriltagCamera");
  private final Transform3d transform3d = new DynamicTransform3d();
  /** Creates a new Navigation. */
  public Navigation() {
    drivetrain.resetEncoderRotations();
    poseEstimator = new DifferentialDrivePoseEstimator(
      new DifferentialDriveKinematics(Constants.TRACK_WIDTH),
      imu.getRotation2d(),
      drivetrain.getLeftDistance(),
      drivetrain.getRightDistance(),
      new Pose2d()
    );
    visionEstimator = new PhotonPoseEstimator(
      Constants.APRILTAG_LAYOUT,
      PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
      camera,
      transform3d
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
  /** Runs once every 20ms. */
  @Override
  public void periodic() {
    poseEstimator.update(
      imu.getRotation2d(),
      drivetrain.getLeftDistance(),
      drivetrain.getRightDistance()
    );
    visionEstimator.setReferencePose(poseEstimator.getEstimatedPosition());
    //var results = visionEstimator.update();
    /*if (results.isPresent())
    {
      poseEstimator.addVisionMeasurement(
        results.get().estimatedPose.toPose2d(),
        results.get().timestampSeconds
      );
    }*/
  }
}