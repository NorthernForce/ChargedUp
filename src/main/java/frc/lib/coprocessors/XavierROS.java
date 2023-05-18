package frc.lib.coprocessors;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.coprocessors.ros.ROSPackage;

public class XavierROS implements Coprocessor {
    private final ArrayList<ROSPackage> rosPackages;
    private final NetworkTableInstance instance;
    private final NetworkTableEntry status;
    private boolean initializedPackagesYet = false;
    public XavierROS()
    {
        rosPackages = new ArrayList<>();
        instance = NetworkTableInstance.create();
        instance.setServerTeam(172);
        instance.startClient4("robot");
        status = instance.getEntry("status");
    }
    @Override
    public Status getStatus() {
        switch (status.getString("offline"))
        {
        case "offline":
            return Status.OFFLINE;
        case "online":
            return Status.ONLINE;
        default:
            return Status.FAILING;
        }
    }
    public void addPackage(ROSPackage pkg)
    {
        rosPackages.add(pkg);
    }
    @Override
    public NetworkTable getTable(String tableName) {
        return instance.getTable(tableName);
    }
    @Override
    public void update() {
        if (getStatus() == Status.ONLINE)
        {
            if (!initializedPackagesYet)
            {
                initializedPackagesYet = true;
                for (var pkg : rosPackages)
                {
                    pkg.initialize();
                }
            }
            for (ROSPackage pkg : rosPackages)
            {
                pkg.update();
            }
        }
    }
}
