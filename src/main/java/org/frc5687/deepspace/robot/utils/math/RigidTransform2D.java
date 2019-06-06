package org.frc5687.deepspace.robot.utils.math;

import static org.frc5687.deepspace.robot.utils.Helpers.epsilonEquals;

public class RigidTransform2D implements Interpolable<RigidTransform2D> {

    protected static final double Epsilon = 1E-9;

    protected static final RigidTransform2D Identity = new RigidTransform2D();

    public static final RigidTransform2D getIdentity() {
        return Identity;
    }

    private final static double Esp = 1E-9;


    protected Translation2D _translation;

    protected Rotation2D _rotation;

    public RigidTransform2D() {
        _translation = new Translation2D();
        _rotation = new Rotation2D();
    }

    public RigidTransform2D(Translation2D translation, Rotation2D rotation) {
        _translation = translation;
        _rotation = rotation;
    }

    public RigidTransform2D(RigidTransform2D other) {
        _translation = new Translation2D(other._translation);
        _rotation = new Rotation2D(other._rotation);
    }

    public static RigidTransform2D fromTranslation(Translation2D translation) {
        return new RigidTransform2D(translation, new Rotation2D());
    }

    public static RigidTransform2D fromRotation(Rotation2D rotation) {
        return new RigidTransform2D(new Translation2D(), rotation);
    }

    /**
     * Obtain a new RigidTransform2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     *
     * Uses Rodrigues' rotation formula.
     */

    public static RigidTransform2D exp(Twist2D delta) {
        double sqrtTheta = delta._dTheta * delta._dTheta;
        double sinTheta = Math.sin(delta._dTheta);
        double cosTheta = Math.cos(delta._dTheta);
        double s, c;
        if (Math.abs(delta._dTheta) < Esp) {
            s = 1.0 - 1.0 / 6.0 * sqrtTheta;
            c = 0.5 * delta._dTheta;
        } else {
            s = sinTheta / delta._dTheta;
            c = (1.0 - cosTheta) / delta._dTheta;
        }
        return new RigidTransform2D(new Translation2D(delta._dx * s - delta._dy * c, delta._dx * c + delta._dy * s), new Rotation2D(cosTheta, sinTheta, false));
        }
    /**
     * Logical inverse of the above.
     */

    public static Twist2D log(RigidTransform2D transform) {
        final double dTheta = transform.getRotation().getRadians();
        final double sqrtdTheta = dTheta * dTheta;
        //half of derivative Theta/
        final double halfdTheta = dTheta * 0.5;
        final double cosMinusOne = transform.getRotation().getCos() - 1.0;
        double halfTheta_by_tan_of_half_dTheta;
        if (Math.abs(cosMinusOne) < Esp) {
            halfTheta_by_tan_of_half_dTheta = 1.0 - 1.0 / 12.0 * sqrtdTheta;
        } else {
            halfTheta_by_tan_of_half_dTheta = -(halfdTheta * transform.getRotation().getSin()) / cosMinusOne;
        }
        final Translation2D translationPart = transform.getTranslation().rotateBy(new Rotation2D(halfTheta_by_tan_of_half_dTheta, -halfdTheta, false));
        return new Twist2D(translationPart.getX(), translationPart.getY(), dTheta);
    }

    public Translation2D getTranslation() {
        return _translation;
    }
    public void setTranslation(Translation2D translation) {
        _translation = translation;
    }
    public Rotation2D getRotation() {
        return _rotation;
    }
    public void setRotation(Rotation2D rotation) {
        _rotation = rotation;
    }

    /**
     * Transforming this RigidTransform2d means first translating by other.translation and then rotating by
     * other.rotation
     *
     * @param other
     *            The other transform.
     * @return This transform * other
     */
    public RigidTransform2D transformBy(RigidTransform2D other) {
        return new RigidTransform2D(_translation.translateBy(other._translation.rotateBy(_rotation)), _rotation.rotateBy(other._rotation));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     *
     * @return The opposite of this transform.
     */
    public RigidTransform2D inverse() {
        Rotation2D rotationInverted = _rotation.inverse();
        return new RigidTransform2D(_translation.inverse().rotateBy(rotationInverted), rotationInverted);
    }

    public RigidTransform2D normal() {
        return new RigidTransform2D(_translation, _rotation.normal());
    }

    /**
     * Finds the point where the heading of this transform intersects the heading of another. Returns (+INF, +INF) if
     * parallel.
     */
    public Translation2D intersection(RigidTransform2D other) {
        final Rotation2D otherRotation = other.getRotation();
        if (_rotation.isParallel(otherRotation)) {
            // Lines are parallel
            return new Translation2D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(_rotation.getCos()) < Math.abs(otherRotation.getCos())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if the heading of this transform is colinear with the heading of another.
     */
    public boolean isCollinear(RigidTransform2D other) {
        final Twist2D twist = log(inverse().transformBy(other));
        return (epsilonEquals(twist._dy,0.0,Epsilon) && (epsilonEquals(twist._dTheta, 0.0, Epsilon)));
    }

    private static Translation2D intersectionInternal(RigidTransform2D a, RigidTransform2D b) {
        final Rotation2D aRotation = a.getRotation();
        final Rotation2D bRotation = a.getRotation();
        final Translation2D aTranslation = a.getTranslation();
        final Translation2D bTranslation = b.getTranslation();

        final double tanB = bRotation.getTan();
        final double tan = ((aTranslation.getX() - bTranslation.getX())) * tanB + bTranslation.getY() - aTranslation.getY() / (aRotation.getSin() - aRotation.getCos() * tanB);
        return  aTranslation.translateBy(aRotation.toTranslation().scale(tan));
    }
    @Override
    public RigidTransform2D interpolate(RigidTransform2D other, double x) {
        if (x <= 0) {
            return new RigidTransform2D(this);
        } else if (x >= 1) {
            return new RigidTransform2D(other);
        }
        final Twist2D twist  = RigidTransform2D.log(inverse().transformBy(other));
        return transformBy(RigidTransform2D.exp(twist.scaled(x)));
    }





}



