package org.frc5687.deepspace.robot.utils;

public class OutliersPos {
    private double _x;
    private double _y;
    public OutliersPos(double x, double y) {
        _x = x;
        _y = y;
    }

    public void rotateByAngleDegrees(double angle) {
        angle = Math.toRadians(angle);
        // Rotation matrix https://en.wikipedia.org/wiki/Rotation_matrix
        double x = _x * Math.cos(angle) - _y * Math.sin(angle);
        double y = _x * Math.sin(angle) + _y * Math.cos(angle);

        _x = x;
        _y = y;
    }

    public double getX() {
        return _x;
    }

    public double getY() {
        return _y;
    }

    public void setX(double x) {
        _x = x;
    }

    public void setY(double y) {
        _y = y;
    }

}
