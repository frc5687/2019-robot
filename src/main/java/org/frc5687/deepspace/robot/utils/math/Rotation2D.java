package org.frc5687.deepspace.robot.utils.math;

public class Rotation2D implements Interpolable<Rotation2D> {

    private double _cos;
    private double _sin;

    public Rotation2D() {
        _cos = 1.0;
        _sin = 0.0;
    }
    public Rotation2D(double cos, double sin) {
        _cos = cos;
        _sin = sin;
    }
    public Rotation2D(Rotation2D other){
        _cos = other.getCos();
        _sin = other.getSin();
    }

    public Rotation2D(double cos, double sin, boolean normalize){
        _cos = cos;
        _sin = sin;
        if (normalize) {
            normalize();
        }
    }

    public static Rotation2D fromDegrees(double degrees) {
        return Rotation2D.fromRadians(Math.toRadians(degrees));
    }

    public static Rotation2D fromRadians(double radians) {
        return new Rotation2D(Math.cos(radians), Math.sin(radians));
    }
    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    public double getRadians() {
        return Math.atan2(_sin, _cos);
    }

    public double getCos() {
        return _cos;
    }

    public double getSin() {
        return _sin;
    }
    public Rotation2D inverse() {
        return new Rotation2D(_cos, -_sin);
    }

    public Rotation2D flip() {
        return new Rotation2D(-_cos, -_sin);
    }

    public void normalize() {
        double magnitude = Math.hypot(_cos, _sin);
        if (magnitude > 1E-6) {
            _cos /= magnitude;
            _sin /= magnitude;
        } else {
            _cos = 1.0;
            _sin = 0.0;
        }
    }
    public Translation2D toTranslation() {
        return new Translation2D(_cos, _sin);
    }

    public Rotation2D rotateBy(Rotation2D rotation){
        return new Rotation2D(_cos * rotation.getCos() - _sin * rotation.getSin(), _sin * rotation.getCos() + _cos * rotation.getSin(), true);
    }

    @Override
    public Rotation2D interpolate(Rotation2D other, double x) {
        Rotation2D delta = inverse().rotateBy(other);
        return rotateBy(Rotation2D.fromRadians(delta.getRadians() * x));
    }

}
