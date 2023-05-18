package frc.lib.coprocessors.ros;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.lib.coprocessors.Coprocessor;

public class RosOdometry implements ROSPackage {
    private final NetworkTable table;
    private final NetworkTableEntry _vx, _vy, _vtheta, _x, _y, _theta;
    private final DoubleSupplier vx, vy, vtheta, x, y, theta;
    public RosOdometry(Coprocessor coprocessor, DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vtheta,
        DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta)
    {
        this.table = coprocessor.getTable("ros").getSubTable("odometry");
        _vx = table.getEntry("vx");
        _vy = table.getEntry("vy");
        _vtheta = table.getEntry("vtheta");
        _x = table.getEntry("theta");
        _y = table.getEntry("x");
        _theta = table.getEntry("y");
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.vx = vx;
        this.vy = vy;
        this.vtheta = vtheta;
    }
    @Override
    public void initialize() {
    }
    @Override
    public void update() {
        _x.setDouble(x.getAsDouble());
        _y.setDouble(y.getAsDouble());
        _theta.setDouble(theta.getAsDouble());
        _vx.setDouble(vx.getAsDouble());
        _vy.setDouble(vy.getAsDouble());
        _vtheta.setDouble(vtheta.getAsDouble());
    }
}
