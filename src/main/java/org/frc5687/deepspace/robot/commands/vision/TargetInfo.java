package org.frc5687.deepspace.robot.commands.vision;

public class TargetInfo {
    protected double _x = 1.0;
    protected double _y;
    protected double _z;

    public TargetInfo(double y, double z) {
        this._y = y;
        this._z = z;
    }

    public double getX() {
        return _x;
    }

    public double getY() {
        return _y;
    }

    public double getZ() {
        return _z;
    }
}
