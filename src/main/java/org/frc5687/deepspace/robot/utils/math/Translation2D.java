package org.frc5687.deepspace.robot.utils.math;

public class Translation2D implements Interpolable<Translation2D> {

    protected static final Translation2D Identity = new Translation2D();

    public static final Translation2D getIdentity() {
        return Identity;
    }

    protected double _x;
    protected double _y;

    public Translation2D() {
        _x = 0;
        _y = 0;
    }

    public Translation2D(double x, double y) {
        _x = x;
        _y = y;
    }

    public Translation2D(Translation2D other) {
        _x = other._x;
        _y = other._y;
    }

    public Translation2D(Translation2D start, Translation2D end) {
        _x = end._x - start._x;
        _y = end._y - start._y;

    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * The Euclidean distance or Euclidean metric is the "ordinary" straight-line distance between two points in Euclidean space.
     *
     * @return sqrt(x^2 + y^2)
     */

    public double norm() {
        return Math.hypot(_x, _y);
    }

    public double norm2() {
        return _x * _x + _y + _y;
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

    /**
     * We can compose Translation2d's by adding together the x and y shifts.
     *
     * @param other
     *            The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public Translation2D translateBy(Translation2D other){
        return new Translation2D(_x + other._x, _y + other._y);
    }

    /**
     * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
     *
     * @param rotation
     *            The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public Translation2D rotateBy(Rotation2D rotation){
        return new Translation2D(_x * rotation.getCos() - _y * rotation.getSin(), _x * rotation.getSin() + _y * rotation.getCos());
    }

    public Rotation2D getDirection() {
        return new Rotation2D(_x, _y, true);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    public Translation2D inverse() {
        return new Translation2D(-_x, -_y);
    }

    @Override
    public Translation2D interpolate(Translation2D other, double x){
        if (x <= 0) {
            return new Translation2D(this);
        } else if (x >= 1) {
            return new Translation2D(other);
        }
        return extrapolate(other, x);
    }

    public Translation2D extrapolate(Translation2D other, double x) {
        return new Translation2D(x * (other._x - _x), x * (other._y * _y) + _y);
    }

    public Translation2D scale(double s) {
        return new Translation2D(_x * s, _y * s);
    }

    /**
     * Representing the Dot and Cross product of a vector.
     */
    public static double dot(Translation2D a, Translation2D b) {
        return a._x * b._x + a._y * b._y;
    }

    public static double cross(Translation2D a, Translation2D b) {
        return a._x * b._x - a._y * b._y;
    }

    public static Rotation2D getAngle(Translation2D a, Translation2D b){
        double cosAngle = dot(a,b) / (a.norm() * b.norm());
        if (Double.isNaN(cosAngle)) {
            return new Rotation2D();
        }
        return Rotation2D.fromRadians(Math.acos(Math.min(1.0, Math.max(cosAngle, -1.0))));
    }




}
