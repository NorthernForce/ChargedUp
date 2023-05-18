package frc.lib.coprocessors.ros;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.coprocessors.Coprocessor;

public class RosNavigation implements ROSPackage {
    private final NetworkTableEntry targetPoseEntry, vx, vy, vtheta;
    public class RosNavigationCommand extends CommandBase
    {
        private final Pose2d targetPose;
        private final Consumer<ChassisSpeeds> drive;
        private final Supplier<Pose2d> currentPose;
        private final double linearTolerance, angularTolerance;
        public RosNavigationCommand(Pose2d targetPose, Consumer<ChassisSpeeds> drive, Supplier<Pose2d> currentPose,
            double linearTolerance, double angularTolerance)
        {
            this.targetPose = targetPose;
            this.drive = drive;
            this.currentPose = currentPose;
            this.linearTolerance = linearTolerance;
            this.angularTolerance = angularTolerance;
        }
        @Override
        public void initialize()
        {
            targetPoseEntry.setDoubleArray(new double[]{
                targetPose.getX(),
                targetPose.getY(),
                targetPose.getRotation().getDegrees()
            });
        }
        @Override
        public void execute()
        {
            drive.accept(new ChassisSpeeds(
                vx.getDouble(0),
                vy.getDouble(0),
                vtheta.getDouble(0)
            ));
        }
        @Override
        public boolean isFinished()
        {
            return currentPose.get().getTranslation().getDistance(targetPose.getTranslation()) < linearTolerance
                && Math.abs(currentPose.get().getRotation().minus(targetPose.getRotation()).getDegrees())
                    < angularTolerance;
        }
    }
    public RosNavigation(Coprocessor coprocessor)
    {
        NetworkTable table = coprocessor.getTable("ros").getSubTable("navigation");
        vx = table.getEntry("vx");
        vy = table.getEntry("vy");
        vtheta = table.getEntry("vtheta");
        targetPoseEntry = table.getEntry("targetPose");
    }
    @Override
    public void initialize() {
    }
    @Override
    public void update() {
    }
}
