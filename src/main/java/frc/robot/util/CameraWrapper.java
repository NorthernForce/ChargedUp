package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public interface CameraWrapper {
    /**
     * Returns the Transform3d (X,Y,Z comp and Pitch, Roll, Yaw comp)
     * @return Transform 3d: Camera to Robot Center on floor
     */
    public default Transform3d getTransform()
    {
        return new Transform3d();
    }
    /**
     * Set the pipeline index
     * @param index index of the pipeline.
     */
    public void setPipelineIndex(int index);
    /**
     * Gets the index of the pipeline
     * @return index of the current pipeline
     */
    public int getPipelineIndex();
    /**
     * Gets whether a target exists
     * @return whether the camera sees a target
     */
    public boolean hasTarget();
    /**
     * Gets yaw to target
     * @return target yaw
     */
    public Rotation2d getTargetYaw();
    /**
     * Gets pitch to target
     * @return target pitch
     */
    public Rotation2d getTargetPitch();
    /**
     * Gets target distance
     * @return target distance in meters
     */
    public double getTargetDistance(double estimatedTargetHeight);
}