package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.cameras.PhotonCameraWrapper;
import frc.robot.Constants;
import frc.robot.FieldConstants;

import org.photonvision.EstimatedRobotPose;

import static frc.robot.RobotContainer.*;

import java.util.List;

/** Simple subsystem to keep track of the current location of the robot. */
public class Navigation extends SubsystemBase {
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final List<PhotonCameraWrapper> cameras = activeChassis.getPhotonCameras();
  private final Field2d field = new Field2d();
  /** Creates a new Navigation. */
  public Navigation() {
    assert drivetrain != null;
    assert imu != null;
    drivetrain.resetEncoderRotations();
    poseEstimator = new DifferentialDrivePoseEstimator(
      new DifferentialDriveKinematics(Constants.DrivetrainConstants.TRACK_WIDTH),
      imu.getRotation2d(),
      drivetrain.getLeftDistance(),
      drivetrain.getRightDistance(),
      new Pose2d()
    );
    for (var camera : cameras)
    {
      camera.setPipelineIndex(0);
    }
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
  /**
   * Gets the nearest scoring location
   * @return scoring location (Translation3d)
   */
  public Translation3d getNearestScoringLocation()
  {
    Pose2d pose = getPose2d();
    if (DriverStation.getAlliance() == Alliance.Blue)
    {
      Translation3d nearest = FieldConstants.BLUE_CONE_PLACEMENT_LOCATIONS[0];
      for (var loc : FieldConstants.BLUE_CONE_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : FieldConstants.BLUE_CUBE_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : FieldConstants.BLUE_FLOOR_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : FieldConstants.BLUE_SHELF_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : FieldConstants.BLUE_GAME_PIECE_AUTO_LOCATIONS)
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
      Translation3d nearest = FieldConstants.RED_CONE_PLACEMENT_LOCATIONS[0];
      for (var loc : FieldConstants.RED_CONE_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : FieldConstants.RED_CUBE_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : FieldConstants.RED_FLOOR_PLACEMENT_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : FieldConstants.RED_SHELF_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      for (var loc : FieldConstants.RED_GAME_PIECE_AUTO_LOCATIONS)
      {
        if (loc.toTranslation2d().getDistance(pose.getTranslation()) < nearest.toTranslation2d().getDistance(pose.getTranslation()))
        {
          nearest = loc;
        }
      }
      return nearest;
    }
  }

  /** Runs once every 20ms. */
  @Override
  public void periodic() {
    poseEstimator.update(
      imu.getRotation2d(),
      drivetrain.getLeftDistance(),
      drivetrain.getRightDistance()
    );

    for (var camera : cameras)
    {
      EstimatedRobotPose pose;
      camera.updateLatestResult();
      if ((pose = camera.estimatePose(poseEstimator.getEstimatedPosition())) != null)
      {
        poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
      }
    }

    field.setRobotPose(poseEstimator.getEstimatedPosition());

  }
}  