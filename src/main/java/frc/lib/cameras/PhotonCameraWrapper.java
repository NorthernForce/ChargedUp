package frc.lib.cameras;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import frc.robot.Constants.DrivetrainConstants;

public class PhotonCameraWrapper implements CameraWrapper {
    private final Transform3d transformToCenter;
    private final PhotonPoseEstimator visionEstimator;
    private EstimatedRobotPose lastEstimatedRobotPose;
    private final PhotonCamera camera;
    public PhotonCameraWrapper(String name, Transform3d transform)
    {
        camera = new PhotonCamera(name);
        transformToCenter = transform;
        visionEstimator = new PhotonPoseEstimator(FieldConstants.APRILTAG_LAYOUT, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, camera, transform);
        lastEstimatedRobotPose = null;
    }
    /**
     * Returns the Transform3d (X,Y,Z comp and Pitch, Roll, Yaw comp)
     * @return Transform 3d: Camera to Robot Center on floor
     */
    @Override
    public Transform3d getTransform()
    {
        return transformToCenter;
    }
    /**
     * Set the pipeline index
     * @param index index of the pipeline.
     */
    @Override
    public void setPipelineIndex(int index) {
        camera.setPipelineIndex(index);
    }
    /**
     * Gets the index of the pipeline
     * @return index of the current pipeline
     */
    @Override
    public int getPipelineIndex() {
        return camera.getPipelineIndex();
    }
    /**
     * Gets whether a target exists
     * @return whether the camera sees a target
     */
    @Override
    public boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }
    /**
     * Gets yaw to target
     * @return target yaw
     */
    @Override
    public Rotation2d getTargetYaw() {
        return Rotation2d.fromDegrees(camera.getLatestResult().getBestTarget().getYaw());
    }
    /**
     * Gets pitch to target
     * @return target pitch
     */
    @Override
    public Rotation2d getTargetPitch() {
        return Rotation2d.fromDegrees(camera.getLatestResult().getBestTarget().getPitch());
    }
    /**
     * Gets target distance
     * @return target distance in meters
     */
    @Override
    public double getTargetDistance(double estimatedTargetHeight) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            transformToCenter.getZ(),
            estimatedTargetHeight,
            transformToCenter.getRotation().getY(),
            getTargetPitch().getRadians()
        );
    }
    /**
     * Uses the apriltag ability to calculate the position of the robot
     * @param referencePose the current computed position by the encoders
     * @return the position of the robot estimated by the camera
     */
    public EstimatedRobotPose estimatePose(Pose2d referencePose)
    {
        visionEstimator.setReferencePose(referencePose);
        var results = visionEstimator.update();
        if (lastEstimatedRobotPose != null && results.isPresent())
        {
            double dtime = results.get().timestampSeconds - lastEstimatedRobotPose.timestampSeconds;
            double distance = results.get().estimatedPose.toPose2d().getTranslation().getDistance(
                lastEstimatedRobotPose.estimatedPose.toPose2d().getTranslation()
            );
            double speed = distance / dtime;
            if (speed <= DrivetrainConstants.MAX_POSSIBLE_SPEED)
            {
                lastEstimatedRobotPose = results.get();
                return lastEstimatedRobotPose;
            }
            else
            {
                return null;
            }
        }
        else if (results.isPresent())
        {
            lastEstimatedRobotPose = results.get();
            return lastEstimatedRobotPose;
        }
        else return null;
    }
}
