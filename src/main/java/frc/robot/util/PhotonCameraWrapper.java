package frc.robot.util;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class PhotonCameraWrapper implements CameraWrapper {
    private final Transform3d transformToCenter;
    private final PhotonPoseEstimator visionEstimator;
    private final PhotonCamera camera;
    public PhotonCameraWrapper(String name, Transform3d transform)
    {
        camera = new PhotonCamera(name);
        transformToCenter = transform;
        visionEstimator = new PhotonPoseEstimator(Constants.APRILTAG_LAYOUT, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, camera, transform);
    }
    @Override
    public Transform3d getTransform()
    {
        return transformToCenter;
    }
    @Override
    public void setPipelineIndex(int index) {
        camera.setPipelineIndex(index);
    }
    @Override
    public int getPipelineIndex() {
        return camera.getPipelineIndex();
    }
    @Override
    public boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }
    @Override
    public Rotation2d getTargetYaw() {
        return Rotation2d.fromDegrees(camera.getLatestResult().getBestTarget().getYaw());
    }
    @Override
    public Rotation2d getTargetPitch() {
        return Rotation2d.fromDegrees(camera.getLatestResult().getBestTarget().getPitch());
    }
    @Override
    public double getTargetDistance(double estimatedTargetHeight) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            transformToCenter.getZ(),
            estimatedTargetHeight,
            transformToCenter.getRotation().getX(),
            getTargetPitch().getRadians()
        );
    }
    public EstimatedRobotPose estimatePose(Pose2d referencePose)
    {
        visionEstimator.setReferencePose(referencePose);
        var results = visionEstimator.update();
        if (results.isPresent()) return results.get();
        else return null;
    }
}
