package org.frc5687.deepspace.robot.utils.math;

import javax.xml.crypto.dsig.TransformService;

public class Translation2D implements Interpolable<Translation2D> {

    private double _x;
    private double _y;

    public Translation2D() {
        _x = 0;
        _y = 0;
    }

    public Translation2D(double x, double y) {
        this._x = x;
        this._y = y;
    }

    public Rotation2D getAngleFromOffsetFromYAxis(Translation2D offset) {
        return offset.getAngleFromAxis(this);
    }

    public Rotation2D getAngleFromOffset(Translation2D offset) {
        return offset.getAngle(this);
    }

    public double getDistanceTo(Translation2D nextPoint) {
        return Math.sqrt(Math.pow((_x - nextPoint.getX()), 2) + Math.pow(_y - nextPoint.getY(), 2));
    }

    public Rotation2D getAngleFromAxis(Translation2D nextPoint) {
        double angleOffset = Math.asin((_x - nextPoint.getX()) / getDistanceTo(nextPoint));
        return Rotation2D.fromRadians(angleOffset);
    }

    public Rotation2D getAngle(Translation2D nextPoint) {
        double angleOffset = Math.atan2(nextPoint.getY() - _y, nextPoint.getX() - _x);
        return Rotation2D.fromRadians(angleOffset);
    }

    public double getX() {
        return _x;
    }

    public double getY() {
        return _y;
    }

    public Translation2D inverse() {
        return new Translation2D(-_x, -_y);
    }

    public Translation2D rotateBy(Rotation2D rotationMat) {
        double x2 = _x * rotationMat.cos() - _y * rotationMat.sin();
        double y2 = _y * rotationMat.sin() - _x * rotationMat.cos();
        return new Translation2D(x2, y2);
    }

    public Translation2D translateBy(Translation2D delta) {
        return new Translation2D(_x + delta.getX(), _y + delta.getY());
    }

    @Override
    public Translation2D interpolate(Translation2D other, double percentage) {
        Translation2D delta = new Translation2D(this.getX() - other.getX(), this.getY() - other.getY());
        return new Translation2D(this.getX() + delta.getX() * percentage, this.getY() + delta.getY() * percentage);
    }
}
