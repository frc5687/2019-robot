package org.frc5687.deepspace.robot.utils.math;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
 * new RigidTransform2d's from a Twist2d and visa versa.
 *
 * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
 */
public class Twist2D {
    protected static final Twist2D Identity = new Twist2D(0.0,0.0,0.0);

    public static final Twist2D getIdentity() {
        return Identity;
    }

    public final double _dx;
    public final double _dy;
    public final double _dTheta; // In Radians.

    public Twist2D(double dx, double dy, double dTheta) {
        _dx = dx;
        _dy = dy;
        _dTheta = dTheta;
    }

    public Twist2D scaled(double scale) {
        return new Twist2D(_dx * scale, _dy * scale, _dTheta * scale);
    }

}
