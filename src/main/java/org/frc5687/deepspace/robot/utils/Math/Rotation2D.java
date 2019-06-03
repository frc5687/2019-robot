package org.frc5687.deepspace.robot.utils.Math;

import static org.frc5687.deepspace.robot.utils.Helpers.epsilonEquals;

public class Rotation2D implements Interpolable<Rotation2D> {

    public static final Rotation2D Identity = new Rotation2D();

    public static final Rotation2D getIdentity() {
        return Identity;
    }

    //determine whether the number is small enough to be insignificant.
    protected static final double Epsilon = 1E-9;

    //cosine angle
    protected double cosAngle;
    //sin angle
    protected double sinAngle;

    public Rotation2D() {
        this(1,0,false);
    }

    public Rotation2D(double x, double y, boolean normalize) {
        cosAngle = x;
        sinAngle = y;
        if(normalize) {
            normalize();
        }
    }

    public Rotation2D(Rotation2D other) {
        cosAngle = other.cosAngle;
        sinAngle = other.sinAngle;
    }

    public static Rotation2D fromRadians(double angleRadians) {
        return new Rotation2D(Math.cos(angleRadians), Math.sin(angleRadians), false);
    }

    public static Rotation2D fromDegrees(double angleDegrees){
        return fromRadians(Math.toRadians(angleDegrees));
    }
    /**
     * From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
     * Normalizing forces us to re-scale the sin and cos to reset rounding errors.
     */
    public void normalize() {
        // magnitude is the length of a vector.
        // Math.hypot This method returns sqrt(x2 +y2) without intermediate overflow or underflow
        double magnitude = Math.hypot(cosAngle, sinAngle);
        if (magnitude > Epsilon) {
            cosAngle /= magnitude;
            sinAngle /= magnitude;
        } else {
            cosAngle = 1;
            sinAngle = 0;
        }

    }

    public double getCos() {
        return cosAngle;
    }

    public double getSin() {
        return sinAngle;
    }

    public double getTan() {
        if (Math.abs(cosAngle) < Epsilon) {
            if (sinAngle >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sinAngle / cosAngle;
    }
    public double getRadians() {
        return Math.atan2(sinAngle, cosAngle);
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }
    /**
     * We can rotate this Rotation2d by adding together the effects of it and another rotation.
     *
     * @param other
     *            The other rotation. See: https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    public Rotation2D rotateBy(Rotation2D other) {
        return new Rotation2D(cosAngle * other.cosAngle - sinAngle * other.sinAngle, cosAngle * other.cosAngle + sinAngle * other.sinAngle, true);
    }

    public Rotation2D normal() {
        return new Rotation2D(-sinAngle, cosAngle, false);
    }

    /**
     * The inverse of a Rotation2d "undoes" the effect of this rotation.
     *
     * @return The opposite of this rotation.
     */
    public Rotation2D inverse() {
        return new Rotation2D(cosAngle, -sinAngle, false);
    }

    /**
     * Checking if two values are withing the error of Epsilon.
     */
    public boolean isParallel(Rotation2D other) {
        return epsilonEquals(Translation2D.cross(toTranslation(), other.toTranslation()), 0.0, Epsilon);
    }
    public Translation2D toTranslation() {
        return new Translation2D(cosAngle, sinAngle);
    }

    public Rotation2D interpolate(Rotation2D other, double x) {
        if (x <= 0) {
            return new Rotation2D(this);
        } else if (x >= 1){
            return new Rotation2D(other);
        }
        double angleDifference = inverse().rotateBy(other).getRadians();
        return this.rotateBy(Rotation2D.fromRadians(angleDifference * x));
    }
}
