package frc.lib.coprocessors;

import edu.wpi.first.networktables.NetworkTable;

public interface Coprocessor {
    public static enum Status
    {
        OFFLINE,
        FAILING,
        ONLINE
    }
    public void update();
    public Status getStatus();
    public NetworkTable getTable(String tableName);
}