package frc.lib.coprocessors.ros;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.lib.coprocessors.Coprocessor;

public class RosOdometry implements ROSPackage {
    private final NetworkTable table;
    private final NetworkTableEntry _vx, _vy, _vtheta, _x, _y, _theta;
    private final Supplier<ChassisSpeeds> velocity;
    private final Supplier<Pose2d> pose;
    public RosOdometry(Coprocessor coprocessor, Supplier<ChassisSpeeds> velocity, Supplier<Pose2d> pose)
    {
        this.table = coprocessor.getTable("ros").getSubTable("odometry");
        _vx = table.getEntry("vx");
        _vy = table.getEntry("vy");
        _vtheta = table.getEntry("vtheta");
        _x = table.getEntry("theta");
        _y = table.getEntry("x");
        _theta = table.getEntry("y");
        this.velocity = velocity;
        this.pose = pose;
    }
    @Override
    public void initialize() {
    }
    @Override
    public void update() {
        var pose = this.pose.get();
        var velocity = this.velocity.get();
        _x.setDouble(pose.getX());
        _y.setDouble(pose.getY());
        _theta.setDouble(pose.getRotation().getDegrees());
        _vx.setDouble(velocity.vxMetersPerSecond);
        _vy.setDouble(velocity.vyMetersPerSecond);
        _vtheta.setDouble(Math.toDegrees(velocity.omegaRadiansPerSecond));
    }
}
