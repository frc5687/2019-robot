package org.frc5687.deepspace.robot.utils.math;

public class Rotation2D implements Interpolable<Rotation2D> {

    private double _cos;
    private double _sin;

    public Rotation2D() {
        _cos = 1;
        _sin = 0;
    }

    public Rotation2D(double cos, double sin) {
        this._cos = cos;
        this._sin = sin;
    }

    public Rotation2D(double cos, double sin, boolean normalize) {
        this._cos = cos;
        this._sin = sin;
        if (normalize) {
            normalize();
        }
    }

    public static Rotation2D fromDegrees(double angle) {
        return Rotation2D.fromDegrees(Math.toRadians(angle));
    }


    public static Rotation2D fromRadians(double radians) {
        return new Rotation2D(Math.cos(radians), Math.sin(radians));
    }

    public void normalize() {
        double magnitude = Math.hypot(_cos, _sin);
        if (magnitude > 1E-9) {
            _cos /= magnitude;
            _sin /= magnitude;
        } else {
            _cos = 1;
            _sin = 0;
        }
    }

    public double cos() {
        return _cos;
    }

    public double sin() {
        return _sin;
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    public double getRadians() {
        return Math.atan2(_sin, _cos);
    }

    public Rotation2D inverse() {
        return new Rotation2D(_cos, -_sin);
    }

    public Rotation2D flip() {
        return new Rotation2D(-_cos, -_sin);
    }
    public Rotation2D rotateBy(Rotation2D rotationMat) {
        return new Rotation2D(_cos * rotationMat.cos() - _sin * rotationMat.sin(), _sin * rotationMat.cos() + _cos * rotationMat.sin(), true);
    }

    @Override
    public Rotation2D interpolate(Rotation2D other, double percentage) {
        Rotation2D diff = inverse().rotateBy(other);
        return rotateBy(Rotation2D.fromRadians(diff.getRadians() * percentage));
    }






}
