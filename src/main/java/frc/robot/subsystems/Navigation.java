package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import static frc.robot.RobotContainer.*;

/** Simple subsystem to keep track of the current location of the robot. */
public class Navigation extends SubsystemBase {
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final PhotonPoseEstimator visionEstimator;
  private final PhotonCamera camera = new PhotonCamera(Constants.NAVIGATION_CAMERA_NAME);
  private final Field2d field = new Field2d();
  /** Creates a new Navigation. */
  public Navigation() {
    assert drivetrain != null;
    assert imu != null;
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
      PoseStrategy.MULTI_TAG_PNP,
      camera,
      Constants.NAVIGATION_CAMERA_TRANSFORM
    );
    camera.setPipelineIndex(0);
    Shuffleboard.getTab("Autonomous").add("Field", field).withSize(3, 2).withPosition(0, 0);
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
   * Sets the robot position to a pose. This method does not check the validity of the pose.
   * @param pose A Pose2d Object
   */
  public void setRobotPose(Pose2d pose)
  {
    poseEstimator.resetPosition(
      imu.getRotation2d(),
      drivetrain.getLeftDistance(),
      drivetrain.getRightDistance(),
      pose
    );
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
  }
  /**
   * Gets the nearest scoring location
   * @return scoring location (Translation3d)
   */
  public Translation3d getNearestScoringLocation()
  {
    Pose2d pose = getPose2d();
    if (DriverStation.getAlliance() == Alliance.Blue)
    {
      Translation3d nearest = Constants.BLUE_CONE_PLACEMENT_LOCATIONS[0];
      for (var loc : Constants.BLUE_CONE_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : Constants.BLUE_CUBE_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : Constants.BLUE_FLOOR_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : Constants.BLUE_SHELF_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : Constants.BLUE_GAME_PIECE_AUTO_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      return nearest;
    }
    else
    {
      Translation3d nearest = Constants.RED_CONE_PLACEMENT_LOCATIONS[0];
      for (var loc : Constants.RED_CONE_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : Constants.RED_CUBE_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : Constants.RED_FLOOR_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : Constants.RED_SHELF_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : Constants.RED_GAME_PIECE_AUTO_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      return nearest;
    }
  }
}