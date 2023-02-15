package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import static frc.robot.RobotContainer.*;

import java.util.List;

/** Simple subsystem to keep track of the current location of the robot. */
public class Navigation extends SubsystemBase {
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final PhotonPoseEstimator visionEstimator;
  private final PhotonCamera camera = new PhotonCamera(activeChassis.getStringConstant("NAVIGATION_CAMERA_NAME"));
  public static final AprilTagFieldLayout APRILTAG_LAYOUT = new AprilTagFieldLayout(
      List.of(
          new AprilTag(1, new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0, 0, Math.toRadians(180))
          )),
          new AprilTag(2, new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0, 0, Math.toRadians(180))
          )),
          new AprilTag(3, new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(174.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0, 0, Math.toRadians(180))
          )),
          new AprilTag(4, new Pose3d(
              Units.inchesToMeters(636.96),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d(0, 0, Math.toRadians(180))
          )),
          new AprilTag(5, new Pose3d(
              Units.inchesToMeters(14.25),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d(0, 0, Math.toRadians(180))
          )),
          new AprilTag(6, new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(174.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0, 0, Math.toRadians(180))
          )),
          new AprilTag(7, new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0, 0, Math.toRadians(180))
          )),
          new AprilTag(8, new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0, 0, Math.toRadians(180))
          ))
      ), 
      16.4846, 
      8.1026
  );
  /** TODO */
  public static final Pose2d[] BLUE_POSES = new Pose2d[] {
      new Pose2d(),
      new Pose2d(),
      new Pose2d()
  };
  /** TODO */
  public static final Pose2d[] RED_POSES = new Pose2d[] {
      new Pose2d(),
      new Pose2d(),
      new Pose2d()
  };
  private final Field2d field = new Field2d();
  /** Creates a new Navigation. */
  public Navigation() {
    assert drivetrain != null;
    assert imu != null;
    drivetrain.resetEncoderRotations();
    poseEstimator = new DifferentialDrivePoseEstimator(
      new DifferentialDriveKinematics(activeChassis.getDoubleConstant("TRACK_WIDTH")),
      imu.getRotation2d(),
      drivetrain.getLeftDistance(),
      drivetrain.getRightDistance(),
      new Pose2d()
    );
    visionEstimator = new PhotonPoseEstimator(
      APRILTAG_LAYOUT,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
      camera,
      (Transform3d)activeChassis.getObjectConstant("NAVIGATION_CAMERA_TRANSFORM")
    );
    camera.setPipelineIndex(0);
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
    var results = visionEstimator.update();
    if (results.isPresent())
    {
      poseEstimator.addVisionMeasurement(
        results.get().estimatedPose.toPose2d(),
        results.get().timestampSeconds
      );
    }
    field.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putData(field);
  }
}