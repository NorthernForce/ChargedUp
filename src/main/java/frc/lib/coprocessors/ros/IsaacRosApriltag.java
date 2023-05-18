package frc.lib.coprocessors.ros;

import java.util.function.BiConsumer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.lib.coprocessors.Coprocessor;

public class IsaacRosApriltag implements ROSPackage {
    private final NetworkTable table;
    private final NetworkTableEntry data, ids;
    private final BiConsumer<Double, Pose2d> visionEstimate;
    private final AprilTagFieldLayout field;
    private final Transform3d cameraOffset;
    public IsaacRosApriltag(Coprocessor coprocessor, BiConsumer<Double, Pose2d> visionEstimate, AprilTagFieldLayout field,
        Transform3d cameraOffset)
    {
        table = coprocessor.getTable("ros").getSubTable("apriltag");
        this.visionEstimate = visionEstimate;
        this.field = field;
        this.data = table.getEntry("data");
        this.ids = table.getEntry("ids");
        this.cameraOffset = cameraOffset;
    }
    @Override
    public void initialize() {
    }
    @Override
    public void update() {
        long[] ids = this.ids.getIntegerArray(new long[]{});
        double[] data = this.data.getDoubleArray(new double[]{});
        int numTags = ids.length;
        for (int i = 0; i < numTags; i++)
        {
            long tagID = ids[i];
            double timestamp = data[i * 7];
            Transform3d transform = new Transform3d(
                new Translation3d(data[i * 8 + 1], data[i * 8 + 2], data[i * 8 + 3]), 
                new Rotation3d(data[i * 8 + 4], data[i * 8 + 5], data[i * 8 + 6])
            );
            var tag = field.getTagPose((int)tagID);
            if (tag.isPresent())
            {
                visionEstimate.accept(
                    timestamp,
                    tag
                        .get()
                        .transformBy(cameraOffset.inverse())
                        .transformBy(transform.inverse())
                        .toPose2d()
                );
            }
        }
    }
}
