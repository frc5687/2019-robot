package org.frc5687.deepspace.robot.utils.math;


public class Translation2D implements Interpolable<Translation2D> {
    private double _x;
    private double _y;

    public Translation2D() {
        _x = 0;
        _y = 0;
    }

    public Translation2D(double x, double y) {
        _x = x;
        _y = y;
    }

    public Translation2D(final Translation2D other) {
        _x = other._x;
        _y = other._y;
    }

    public double getX() {
        return _x;
    }

    public double getY() {
        return _y;
    }

    public Translation2D rotateBy(Rotation2D rotation) {
        double x = _x * rotation.getCos() - _y * rotation.getSin();
        double y = _x * rotation.getSin() + _y * rotation.getCos();
        return new Translation2D(x, y);
    }
    public Translation2D translateBy(final Translation2D other) {
        return new Translation2D(_x + other._x, _y + other._y);
    }
    public Translation2D inverse() {
        return new Translation2D(-_x, -_y);
    }

    @Override
    public Translation2D interpolate(Translation2D other, double x) {
        Translation2D delta = new Translation2D(this.getX() - other.getX(), this.getY() - other.getY());
        return new Translation2D(this.getX() + delta.getX() * x, this.getY() + delta.getY() * x);
    }
}
