package frc.lib.cameras;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightWrapper implements CameraWrapper {
    private final NetworkTableEntry tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    private final NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    private final NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    private final NetworkTableEntry pipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline");
    private final Transform3d transformToCenter;
    public LimelightWrapper(Transform3d transform)
    {
        transformToCenter = transform;
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
        this.pipeline.setDouble(index);
    }
    /**
     * Gets the index of the pipeline
     * @return index of the current pipeline
     */
    @Override
    public int getPipelineIndex() {
        return (int)pipeline.getDouble(0.0);
    }
    /**
     * Gets whether a target exists
     * @return whether the camera sees a target
     */
    @Override
    public boolean hasTarget() {
        return tv.getDouble(0.0) == 1.0;
    }
    /**
     * Gets yaw to target
     * @return target yaw
     */
    @Override
    public Rotation2d getTargetYaw() {
        return Rotation2d.fromDegrees(tx.getDouble(0.0));
    }
    /**
     * Gets pitch to target
     * @return target pitch
     */
    @Override
    public Rotation2d getTargetPitch() {
        return Rotation2d.fromDegrees(ty.getDouble(0.0));
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
    
}
