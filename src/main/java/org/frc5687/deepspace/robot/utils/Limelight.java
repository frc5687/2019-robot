package org.frc5687.switchbot.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    NetworkTable _table;
    NetworkTableEntry _tx;


    public Limelight() {
        this("limelight");
    }


    public Limelight(String key) {
        _table = NetworkTableInstance.getDefault().getTable(key);
        _tx = _table.getEntry("tx");
    }

    public void enableLEDs() {

    }

    public void disableLEDs() {

    }

    public boolean isTargetSighted() {
        return false;
    }

    public double getHorizontalAngle() {
        return _tx.getDouble(0.0);
    }

}
